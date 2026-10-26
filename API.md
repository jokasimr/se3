# se3 API

## Types

- `vec3`: `STRUCT(x DOUBLE, y DOUBLE, z DOUBLE)`
- `quat`: `STRUCT(w DOUBLE, x DOUBLE, y DOUBLE, z DOUBLE)`
- `vec4`: `STRUCT(w DOUBLE, x DOUBLE, y DOUBLE, z DOUBLE)` (same physical layout as `quat`)
- `W`: `STRUCT(t vec3, q quat)`

`vec4` and `quat` intentionally share the same structure, so DuckDB can accept either where that shape is expected. This is for convenience, not semantic equivalence: use vector functions for vectors and quaternion functions for rotations.

## Convention

```
W(p) = R_q(p + t)
```

A point is translated by `t` first, then rotated by quaternion `q`. Quaternions are assumed to be unit length. Axis-angle constructors normalize the supplied axis; if the axis norm is zero they return `NULL`.

## Behavioral Notes

- Except for `se3_identity()`, all functions propagate `NULL` if any required input or required struct field is `NULL`.
- All calculations use double precision.
- Vector math functions follow floating-point semantics and do not add explicit zero-denominator guards.
- Zero denominators and non-finite inputs can produce `NaN`/`Inf`.
- `vangle` returns the unsigned angle between vectors in radians, in `[0, pi]`. `vcos_angle`, `vsin_angle`, `vtan_angle`, and `vcot_angle` return the corresponding trigonometric ratios without clamping.
- All five angle functions return `NaN` if either vector has zero length or a non-finite component.
- Angle calculations rescale inputs to limit intermediate overflow and underflow. Cancellation and component underflow during scaling remain possible.

## Functions

### `vvec(...) -> vec3 | vec4`
Constructs a vector.

Signatures:
- `vvec(x: DOUBLE, y: DOUBLE, z: DOUBLE) -> vec3`
- `vvec(w: DOUBLE, x: DOUBLE, y: DOUBLE, z: DOUBLE) -> vec4`

Constructs either a `vec3` (`STRUCT(x DOUBLE, y DOUBLE, z DOUBLE)`) or a 4D vector
(`STRUCT(w DOUBLE, x DOUBLE, y DOUBLE, z DOUBLE)`, same layout as `quat`).

### `vadd(a, b) -> vec3 | vec4`
Element-wise addition.

Signatures:
- `vadd(a: vec3, b: vec3) -> vec3`
- `vadd(a: vec4, b: vec4) -> vec4`

### `vsub(a, b) -> vec3 | vec4`
Element-wise subtraction (`a - b`).

Signatures:
- `vsub(a: vec3, b: vec3) -> vec3`
- `vsub(a: vec4, b: vec4) -> vec4`

### `vscale(a, s) -> vec3 | vec4`
Scalar multiplication (`a * s`).

Signatures:
- `vscale(a: vec3, s: DOUBLE) -> vec3`
- `vscale(a: vec4, s: DOUBLE) -> vec4`

### `vdot(a, b) -> DOUBLE`
Dot product.

Signatures:
- `vdot(a: vec3, b: vec3) -> DOUBLE`
- `vdot(a: vec4, b: vec4) -> DOUBLE`

### `vnorm2(a) -> DOUBLE`
Squared Euclidean norm.

Signatures:
- `vnorm2(a: vec3) -> DOUBLE`
- `vnorm2(a: vec4) -> DOUBLE`

### `vnorm(a) -> DOUBLE`
Euclidean norm.

Signatures:
- `vnorm(a: vec3) -> DOUBLE`
- `vnorm(a: vec4) -> DOUBLE`

### `vnormalize(a) -> vec3 | vec4`
Unit normalization (`a / vnorm(a)`).

Signatures:
- `vnormalize(a: vec3) -> vec3`
- `vnormalize(a: vec4) -> vec4`

### `vcross(a: vec3, b: vec3) -> vec3`
3D cross product (`a × b`).

### `vcos_angle(a, b) -> DOUBLE`
Cosine of the angle between vectors:
`vdot(a,b) / (vnorm(a) * vnorm(b))`.
No clamping is applied.

Signatures:
- `vcos_angle(a: vec3, b: vec3) -> DOUBLE`
- `vcos_angle(a: vec4, b: vec4) -> DOUBLE`

### `vsin_angle(a, b) -> DOUBLE`
Sine of the angle between vectors: `A / (vnorm(a) * vnorm(b))`, where
`A = sqrt(sum_{i<j} (a_i*b_j - a_j*b_i)^2)` is the parallelogram area
(the cross-product norm in 3D).

Signatures:
- `vsin_angle(a: vec3, b: vec3) -> DOUBLE`
- `vsin_angle(a: vec4, b: vec4) -> DOUBLE`

### `vtan_angle(a, b) -> DOUBLE`
Tangent of the angle between vectors: `A / vdot(a,b)`, with `A` as in `vsin_angle`.
For nonzero `A`, division by a computed `+0` or `-0` returns `+Inf` or `-Inf`,
respectively. Near orthogonality, dot-product cancellation can cause large
relative error or reverse the sign.

Signatures:
- `vtan_angle(a: vec3, b: vec3) -> DOUBLE`
- `vtan_angle(a: vec4, b: vec4) -> DOUBLE`

### `vcot_angle(a, b) -> DOUBLE`
Cotangent of the angle between vectors: `vdot(a,b) / A`, with `A` as in `vsin_angle`.
For a nonzero dot product, a computed zero area returns infinity with the dot
product's sign.

Signatures:
- `vcot_angle(a: vec3, b: vec3) -> DOUBLE`
- `vcot_angle(a: vec4, b: vec4) -> DOUBLE`

### `vangle(a, b) -> DOUBLE`
Returns the unsigned angle between vectors in radians, in `[0, pi]`:
`atan2(A, vdot(a,b))`, with `A` as in `vsin_angle`.
This can resolve small angles whose cosine rounds to one.

Signatures:
- `vangle(a: vec3, b: vec3) -> DOUBLE`
- `vangle(a: vec4, b: vec4) -> DOUBLE`

### `vproj(a, b) -> vec3 | vec4`
Vector projection of `a` onto `b`.

Signatures:
- `vproj(a: vec3, b: vec3) -> vec3`
- `vproj(a: vec4, b: vec4) -> vec4`

### `vrej(a, b) -> vec3 | vec4`
Vector rejection of `a` from `b`:
`a - vproj(a, b)`.

Signatures:
- `vrej(a: vec3, b: vec3) -> vec3`
- `vrej(a: vec4, b: vec4) -> vec4`

### `quat_from_axis_angle(axis: vec3, th: DOUBLE) -> quat`
Creates a quaternion representing a rotation of `th` radians about an axis.
The axis is normalized internally. If the axis norm is zero, returns `NULL`.

Example:
```sql
SELECT quat_from_axis_angle(
  struct_pack(x:=0.0, y:=0.0, z:=1.0),
  pi()/2.0
);
```

### `qmul(qA: quat, qB: quat) -> quat`
Quaternion multiplication (`qA ⊗ qB`).

Example:
```sql
SELECT qmul(
  quat_from_axis_angle(struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0),
  quat_from_axis_angle(struct_pack(x:=0.0, y:=1.0, z:=0.0), pi()/2.0)
);
```

### `qconj(q: quat) -> quat`
Quaternion conjugate (negates vector part).

Example:
```sql
SELECT qconj(struct_pack(w:=1.0, x:=2.0, y:=3.0, z:=4.0));
```

### `qnorm2(q: quat) -> DOUBLE`
Squared norm of a quaternion. For unit quaternions, this should be `1.0`.

Example:
```sql
SELECT qnorm2(quat_from_axis_angle(struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0));
```

### `se3_identity() -> W`
Returns the identity transform (`t = 0`, `q = (1,0,0,0)`).

Example:
```sql
SELECT se3_identity();
```

### `se3_make(t: vec3, q: quat) -> W`
Constructs a transform from a translation and quaternion.

Example:
```sql
SELECT se3_make(
  struct_pack(x:=1.0, y:=2.0, z:=3.0),
  quat_from_axis_angle(struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0)
);
```

### `se3_from_axis_angle(t: vec3, axis: vec3, th: DOUBLE) -> W`
Constructs a transform from translation and axis-angle rotation.
The axis is normalized internally. If the axis norm is zero, returns `NULL`.

Example:
```sql
SELECT se3_from_axis_angle(
  struct_pack(x:=1.0, y:=0.0, z:=0.0),
  struct_pack(x:=0.0, y:=0.0, z:=1.0),
  pi()/2.0
);
```

### `se3_apply(x, p) -> vec3`
Applies a transformation, translation, or rotation to a point.

Signatures:
- `se3_apply(W: W, p: vec3) -> vec3`  
  Apply full transform.
- `se3_apply(t: vec3, p: vec3) -> vec3`  
  Translate: `p + t`.
- `se3_apply(q: quat, p: vec3) -> vec3`  
  Rotate: `R_q(p)`.

Examples: full transform (translation then rotation), translation, and rotation.
```sql
SELECT se3_apply(
  se3_from_axis_angle(
    struct_pack(x:=1.0, y:=0.0, z:=0.0),
    struct_pack(x:=0.0, y:=0.0, z:=1.0),
    pi()/2.0
  ),
  struct_pack(x:=1.0, y:=0.0, z:=0.0)
);

SELECT se3_apply(
  struct_pack(x:=1.0, y:=2.0, z:=3.0),
  struct_pack(x:=4.0, y:=5.0, z:=6.0)
);

SELECT se3_apply(
  quat_from_axis_angle(struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0),
  struct_pack(x:=1.0, y:=0.0, z:=0.0)
);
```

### `se3_inv(x) -> vec3 | quat | W`
Inverse of a translation, rotation, or transform.

Signatures:
- `se3_inv(t: vec3) -> vec3`  
  Returns `-t`.
- `se3_inv(q: quat) -> quat`  
  Returns the conjugate `q*` (inverse for unit quaternions).
- `se3_inv(W: W) -> W`  
  Returns the inverse transform such that `se3_apply(se3_inv(W), se3_apply(W, p)) = p`.

Examples:
```sql
SELECT se3_inv(struct_pack(x:=1.0, y:=2.0, z:=3.0));
SELECT se3_inv(quat_from_axis_angle(struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0));
SELECT se3_inv(se3_from_axis_angle(struct_pack(x:=1.0, y:=0.0, z:=0.0), struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0));
```

### `se3_compose(A, B) -> vec3 | quat | W`
Composition operator. **Semantics:** apply `B` first, then `A`.

For transforms `W_A = (t_A, q_A)` and `W_B = (t_B, q_B)`, the result has:

```text
q = q_A ⊗ q_B
t = t_B + R_{q_B}^{-1}(t_A)
```

Here `⊗` is quaternion multiplication and `R_{q_B}^{-1}` is the inverse rotation
of `q_B`. The translation formula follows the convention `W(p) = R_q(p + t)`.

Signatures:
- `se3_compose(W2: W, W1: W) -> W`  
  Composition of transforms.
- `se3_compose(t2: vec3, t1: vec3) -> vec3`  
  Translation addition (`t1 + t2`).
- `se3_compose(q2: quat, q1: quat) -> quat`  
  Quaternion multiplication (`q2 ⊗ q1`).
- `se3_compose(q: quat, t: vec3) -> W`  
  Apply translation then rotation (`t` then `q`).
- `se3_compose(t: vec3, q: quat) -> W`  
  Apply rotation then translation (internally converted to our `W` convention).
- `se3_compose(W: W, q: quat) -> W`  
  Apply `q` then `W`.
- `se3_compose(q: quat, W: W) -> W`  
  Apply `W` then `q`.
- `se3_compose(W: W, t: vec3) -> W`  
  Apply `t` then `W`.
- `se3_compose(t: vec3, W: W) -> W`  
  Apply `W` then `t`.

Examples:
```sql
-- Compose two transforms (apply W1 then W2)
SELECT se3_compose(W2, W1);

-- Translation then rotation
SELECT se3_compose(
  quat_from_axis_angle(struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0),
  struct_pack(x:=1.0, y:=0.0, z:=0.0)
);

-- Rotation then translation
SELECT se3_compose(
  struct_pack(x:=1.0, y:=0.0, z:=0.0),
  quat_from_axis_angle(struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0)
);
```
