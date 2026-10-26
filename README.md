# se3

A DuckDB extension for SE(3) rigid transformations, quaternion operations and
3D/4D vector arithmetic.

Transforms are stored as structs containing a translation and a unit quaternion.
They can be composed and applied to 3D points.

See the [API reference](API.md) for types, functions and composition semantics.

## Building

This version targets DuckDB **v2.0.0**.

```sh
git submodule update --init --recursive
GEN=ninja make
```

Build outputs:

- `build/release/duckdb`: CLI with `se3` loaded.
- `build/release/test/unittest`: test runner.
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

Result (roundoff omitted):

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
