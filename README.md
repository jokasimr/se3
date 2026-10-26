# se3

A DuckDB extension for SE(3) rigid transformations, quaternion operations and
3D/4D vector arithmetic, implemented as DuckDB scalar functions.

A transform has six degrees of freedom and is stored as `STRUCT(t vec3, q quat)`:
three translation components `t = (x, y, z)` and a four-component unit quaternion
`q = (w, x, y, z)`, for seven `DOUBLE` values in total.

Sequences of transformations can be composed into a single transform and applied
to 3D points without calling trigonometric functions.

See the [API reference](API.md) for types, functions and composition semantics.

## Building

This version targets DuckDB **v2.0.0**.

```sh
git submodule update --init --recursive
GEN=ninja make
```

Build outputs:

- `build/release/duckdb`: CLI with `se3` loaded.
- `build/release/test/unittest`: test runner with `se3` linked in.
- `build/release/extension/se3/se3.duckdb_extension`: loadable extension.

## Running the extension

Start the CLI with `./build/release/duckdb`.

This example translates a point by `(1, 0, 0)`, then rotates it 90 degrees about Z:

```sql
SELECT se3_apply(
  se3_from_axis_angle(
    struct_pack(x:=1.0, y:=0.0, z:=0.0), -- translation
    struct_pack(x:=0.0, y:=0.0, z:=1.0), -- rotation axis (Z)
    pi()/2.0                             -- 90 degrees
  ),
  struct_pack(x:=1.0, y:=0.0, z:=0.0)    -- point
);
```

The point `(1, 0, 0)` is first translated by `(1, 0, 0)` to `(2, 0, 0)`.
Rotating this translated point 90 degrees about Z gives `(0, 2, 0)`
(roundoff omitted):

```
{x: 0.0, y: 2.0, z: 0.0}
```

Compose transformations with `se3_compose(W2, W1)`: apply `W1` first, then `W2`.

```sql
SELECT se3_apply(
  se3_compose(
    se3_from_axis_angle(struct_pack(x:=0.0, y:=1.0, z:=0.0), struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0), -- W2
    se3_from_axis_angle(struct_pack(x:=1.0, y:=0.0, z:=0.0), struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0)  -- W1
  ),
  struct_pack(x:=1.0, y:=0.0, z:=0.0)
);
```

Result (roundoff omitted):

```
{x: -3.0, y: 0.0, z: 0.0}
```

## Running the tests

```sh
make test
```
