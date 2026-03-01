# se3 API

This document is the source of truth for the extension API.

## Types

- `vec3`: `STRUCT(x DOUBLE, y DOUBLE, z DOUBLE)`
- `quat`: `STRUCT(w DOUBLE, x DOUBLE, y DOUBLE, z DOUBLE)`
- `W`: `STRUCT(t vec3, q quat)`

## Convention

```
W(p) = R_q(p + t)
```

A point is translated by `t` first, then rotated by quaternion `q`. Quaternions and axes are assumed to be unit length; no normalization is performed.

## Functions

### `quat_from_axis_angle(axis: vec3, th: DOUBLE) -> quat`
Creates a quaternion representing a rotation of `th` radians about a **unit** axis.

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

Example:
```sql
SELECT se3_from_axis_angle(
  struct_pack(x:=1.0, y:=0.0, z:=0.0),
  struct_pack(x:=0.0, y:=0.0, z:=1.0),
  pi()/2.0
);
```

### `se3_apply(x, p) -> vec3`
Overloaded apply operator.

Overloads:
- `se3_apply(W: W, p: vec3) -> vec3`  
  Apply full transform.
- `se3_apply(t: vec3, p: vec3) -> vec3`  
  Translate: `p + t`.
- `se3_apply(q: quat, p: vec3) -> vec3`  
  Rotate: `R_q(p)`.

Applies the transform to a point.

Example (rotate + translate):
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

### `se3_inv(x) -> x`
Overloaded inverse operator.

Overloads:
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

### `se3_compose(A, B) -> ...`
Overloaded composition operator. **Semantics:** apply `B` first, then `A`.

Overloads:
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
