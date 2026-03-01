# SPEC: se3 DuckDB Extension (Quaternion SE(3))

## What Was Built
A DuckDB extension named `se3` that implements rigid 3D transformations using unit quaternions and vectorized scalar functions. The API is intentionally small and uses structured types for SQL ergonomics:

- `vec3`: `STRUCT(x DOUBLE, y DOUBLE, z DOUBLE)`
- `quat`: `STRUCT(w DOUBLE, x DOUBLE, y DOUBLE, z DOUBLE)`
- `W`: `STRUCT(t vec3, q quat)`

**Convention (source of truth):**

```
W(p) = R_q(p + t)
```

Translation is applied first, then rotation. Unit quaternions and unit axes are assumed (no normalization in kernels).

### Implemented Scalar Functions
- `quat_from_axis_angle(axis: vec3, th: DOUBLE) -> quat`
- `qmul(qA: quat, qB: quat) -> quat`
- `qconj(q: quat) -> quat`
- `qnorm2(q: quat) -> DOUBLE`

- `se3_identity() -> W`
- `se3_make(t: vec3, q: quat) -> W`
- `se3_from_axis_angle(t: vec3, axis: vec3, th: DOUBLE) -> W`
- `se3_apply(W: W, p: vec3) -> vec3`
- `se3_inv(x) -> x` (overloaded; inverse of vec3/quat/W)
- `se3_compose(A, B) -> ...` (overloaded; apply `B` then `A`)

### Composition Semantics
`se3_compose(A, B)` returns an operator equivalent to applying `B` first, then `A`.

Key formulas under the convention `W(p) = R_q(p + t)`:

- **W ∘ W**:  
  `q = qA ⊗ qB`  
  `t = tB + R_{qB}^{-1} tA`
- **t ∘ W** (apply `W` then translation `t`):  
  `q = qW`  
  `t = tW + R_{qW}^{-1} t`
- **W ∘ t** (apply translation `t` then `W`):  
  `q = qW`  
  `t = tW + t`
- **q ∘ W** (apply `W` then rotation `q`):  
  `q = q ⊗ qW`  
  `t = tW`
- **W ∘ q** (apply rotation `q` then `W`):  
  `q = qW ⊗ q`  
  `t = R_q^{-1} tW`
- **q ∘ t** (apply translation `t` then rotation `q`):  
  `q = q`  
  `t = t`
- **t ∘ q** (apply rotation `q` then translation `t`):  
  `q = q`  
  `t = R_q^{-1} t`

## How It Was Built (Implementation)
The implementation is a set of vectorized DuckDB scalar functions written in C++ and registered in the `se3` extension module. Key implementation choices:

- **Vectorized kernels:** Each function operates on a `DataChunk` and processes all rows in tight loops.
- **Unified access once:** Inputs are converted to `UnifiedVectorFormat` once per input vector and reused.
- **Flat outputs:** Output structs are forced to `FLAT_VECTOR`, and child vectors are written directly to avoid per-row struct construction.
- **Quaternion rotation kernel:** Rotation uses the optimized `t = 2*cross(qv, v)` identity to avoid quaternion multiplication or matrix construction.
- **No normalization/checks:** Unit quaternion and unit axis assumptions are enforced by API contract, not code.
- **Nested struct layout:** Types are `STRUCT`s for SQL clarity; all field extraction is done once per vector and reused in tight loops.
- **Left-apply composition:** `se3_left_apply_translate` uses `R^{-1}` for translation update, and `se3_left_apply_rotate` uses `q2 ⊗ q`.

### Memory Layout and Vectorization Details

- **Struct storage in DuckDB:** A `STRUCT` vector in DuckDB is represented as a parent vector with child vectors for each field. For example, `vec3` is a `STRUCT` with three child `DOUBLE` vectors (`x`, `y`, `z`), and `W` is a `STRUCT` with two children (`t` and `q`), each of which is itself a `STRUCT`.

- **UnifiedVectorFormat usage:** Every input child vector is converted once to `UnifiedVectorFormat` via `ToUnifiedFormat(n, uf)`. This produces:
  - `uf.data`: a pointer to the physical data buffer (or a constant/flat representation).
  - `uf.sel`: a selection vector that maps logical row indices `[0..n)` to physical indices.

  The code does **not** assume the inputs are flat; all accesses are through `sel->get_index(i)` to correctly handle dictionary/constant vectors or filtered selections.

- **Selection vector handling:** For each input child vector, the implementation captures both the data pointer and its selection vector. In the tight loop, it resolves the physical index with `sel->get_index(i)` for each input, then reads the scalar. This preserves correctness with filtered chunks and non-flat inputs.

- **Output vectors are forced to flat:** Output vectors are explicitly set to `FLAT_VECTOR` and their child vectors are also forced to `FLAT_VECTOR`. The code then writes outputs directly into the child data buffers. This avoids:
  - Per-row struct materialization
  - Repeated type dispatch
  - Additional selection vector indirection on output

- **Memory locality considerations:** Output writes are contiguous in flat buffers, which is friendly for CPU caches. Input reads may be strided when selections are non-trivial; using `UnifiedVectorFormat` and direct index mapping ensures correctness while keeping the code branch-free in the hot loop.

- **No intermediate allocations:** All kernels operate with stack locals and direct buffer writes. There are no per-row heap allocations or temporary `Value` objects.

### Null Behavior

- **Current behavior:** The functions do **not** explicitly propagate nulls. They read input buffers via `UnifiedVectorFormat` but do not check validity masks. If any input row contains a NULL in any field, behavior is undefined (may read garbage or produce non-sensical outputs).

- **Implications:** This is acceptable only if the API contract guarantees non-NULL inputs for all fields. If NULLs are possible in production usage, we must add validity checks and set the output validity mask accordingly.

- **Potential improvement path:** If needed, the implementation can be extended to:
  - Inspect `uf.validity` for each input field.
  - Combine validity across all required inputs.
  - Mark the output row invalid (NULL) when any input is NULL.
  - Short-circuit computations for NULL rows.

### Numeric Behavior

- **All math uses `double`.** There is no mixed precision.\n- **Unit assumptions:** No renormalization, so accuracy depends on input quaternions and axes being unit length.

## How It Was Built (Compilation)
Recommended local build (Ninja):

```sh
GEN=ninja make
```

Run tests:

```sh
make test
```

Notes:
- This extension does not require VCPKG.
- In this environment, `ccache` failed to write to `/run/user/1000/ccache-tmp`; building with `CCACHE_DISABLE=1` succeeds.

## Trade-offs Made
- **Quaternion rotation (unit assumption)**: No normalization or validation for performance and simplicity. This assumes callers provide unit quaternions and unit axes.
- **Translate-then-rotate convention**: Chosen for consistency and simpler composition rules in the codebase. It requires `R^{-1}` during left-translation.
- **Structured SQL types**: `STRUCT` types are used for API clarity even though they require field extraction. The overhead is negligible relative to the math kernels.
- **Single compose API**: Instead of left-apply helpers, `se3_compose` provides all combinations via overloading.

## Outstanding Issues / Questions
1. **Axis normalization**: Should `quat_from_axis_angle` normalize the axis to protect against accidental non-unit inputs, or remain strict for performance?
2. **Quaternion normalization**: Should any function renormalize (or validate) quaternions to prevent drift in long composition chains?
3. **Right-apply / composition functions**: Do we need `se3_right_apply_translate`, `se3_right_apply_rotate`, or full `se3_compose(WA, WB)`?
4. **Documentation of conventions**: The chosen convention is clear here; ensure all downstream docs and user examples remain consistent.
5. **ccache config**: If we want to use `ccache` by default, we should set `CCACHE_TEMPDIR` to a writable location or document the required environment.
