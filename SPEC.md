# SPEC: se3 DuckDB Extension

## Scope

`se3` is a DuckDB extension that provides:
- Quaternion utilities
- SE(3) transform construction/application/composition
- Generic vector math on 3D and 4D struct vectors

All kernels are implemented as vectorized scalar functions over DuckDB `DataChunk`s.

## Data Types

- `vec3`: `STRUCT(x DOUBLE, y DOUBLE, z DOUBLE)`
- `quat`: `STRUCT(w DOUBLE, x DOUBLE, y DOUBLE, z DOUBLE)`
- `vec4`: `STRUCT(w DOUBLE, x DOUBLE, y DOUBLE, z DOUBLE)` (same layout as `quat`)
- `W`: `STRUCT(t vec3, q quat)`

Note: `vec4` and `quat` are layout-compatible by design. This is convenient for overload resolution, but they are not semantically equivalent.

## Core Convention

The transform convention is:

```
W(p) = R_q(p + t)
```

`t` is applied first, then rotation `q`.

## Public API

### Vector constructors

- `vvec(x, y, z) -> vec3`
- `vvec(w, x, y, z) -> vec4`

### Vector arithmetic

- `vadd(vec3, vec3) -> vec3`
- `vadd(vec4, vec4) -> vec4`
- `vsub(vec3, vec3) -> vec3`
- `vsub(vec4, vec4) -> vec4`
- `vscale(vec3, DOUBLE) -> vec3`
- `vscale(vec4, DOUBLE) -> vec4`
- `vdot(vec3, vec3) -> DOUBLE`
- `vdot(vec4, vec4) -> DOUBLE`
- `vnorm2(vec3) -> DOUBLE`
- `vnorm2(vec4) -> DOUBLE`
- `vnorm(vec3) -> DOUBLE`
- `vnorm(vec4) -> DOUBLE`
- `vnormalize(vec3) -> vec3`
- `vnormalize(vec4) -> vec4`
- `vcross(vec3, vec3) -> vec3`
- `vcos_angle(vec3, vec3) -> DOUBLE`
- `vcos_angle(vec4, vec4) -> DOUBLE`
- `vangle(vec3, vec3) -> DOUBLE`
- `vangle(vec4, vec4) -> DOUBLE`
- `vproj(vec3, vec3) -> vec3`
- `vproj(vec4, vec4) -> vec4`
- `vrej(vec3, vec3) -> vec3`
- `vrej(vec4, vec4) -> vec4`

### Quaternion functions

- `quat_from_axis_angle(axis: vec3, th: DOUBLE) -> quat`
- `qmul(qA: quat, qB: quat) -> quat`
- `qconj(q: quat) -> quat`
- `qnorm2(q: quat) -> DOUBLE`

### SE(3) functions

- `se3_identity() -> W`
- `se3_make(t: vec3, q: quat) -> W`
- `se3_from_axis_angle(t: vec3, axis: vec3, th: DOUBLE) -> W`
- `se3_apply(W, vec3) -> vec3`
- `se3_apply(vec3, vec3) -> vec3`
- `se3_apply(quat, vec3) -> vec3`
- `se3_inv(vec3) -> vec3`
- `se3_inv(quat) -> quat`
- `se3_inv(W) -> W`
- `se3_compose(W, W) -> W`
- `se3_compose(vec3, vec3) -> vec3`
- `se3_compose(quat, quat) -> quat`
- `se3_compose(quat, vec3) -> W`
- `se3_compose(vec3, quat) -> W`
- `se3_compose(W, quat) -> W`
- `se3_compose(quat, W) -> W`
- `se3_compose(W, vec3) -> W`
- `se3_compose(vec3, W) -> W`

## Composition Semantics

`se3_compose(A, B)` means apply `B` first, then `A`.

Key formulas under `W(p) = R_q(p + t)`:

- `W_A ∘ W_B`:
  - `q = q_A ⊗ q_B`
  - `t = t_B + R_{q_B}^{-1} t_A`
- `t2 ∘ t1`:
  - `t = t1 + t2`
- `q2 ∘ q1`:
  - `q = q2 ⊗ q1`

Mixed compose overloads are implemented to preserve this same ordering rule.

## Null and Numeric Behavior

### Nulls

- `se3_identity()` always returns non-`NULL`.
- All other functions propagate `NULL` if any required input/field is `NULL`.

### Numeric semantics

- Math type is `double` throughout.
- Vector math functions intentionally use regular floating-point semantics:
  - No explicit zero-denominator checks in `vnormalize`, `vcos_angle`, `vangle`, `vproj`, `vrej`
  - These can produce `NaN`/`Inf` for edge inputs
  - `vcos_angle` is not clamped to `[-1, 1]`

### Axis-angle normalization

- `quat_from_axis_angle` and `se3_from_axis_angle` normalize the supplied axis internally.
- If axis norm is zero, output is `NULL`.
- Axis norm uses a robust nested `hypot` computation (`hypot(hypot(x, y), z)`) to reduce underflow/overflow issues for tiny/huge inputs.

## Implementation Notes

- All kernels are vectorized and use `UnifiedVectorFormat`.
- Output struct vectors are forced to `FLAT_VECTOR` and written directly into child buffers.
- No per-row heap allocations or `Value` materialization in hot loops.
- Quaternion rotation in `se3_apply` uses a cross-product form instead of matrix construction.

## Testing Status

Primary coverage is in `test/sql/se3.test` (SQLLogicTest), including:
- Base API behavior and overload coverage
- Null propagation
- Composition/inverse identities
- Vector algebra identities
- Floating-point edge cases (`NaN`/`Inf`)
- Tiny/huge axis normalization for axis-angle constructors
