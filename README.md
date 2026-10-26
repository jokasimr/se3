# se3

A DuckDB extension for SE(3) rigid transformations, quaternion operations and
3D/4D vector arithmetic, implemented as DuckDB scalar functions.

* Sequences of transformations can be combined to a 6 DOF transform stored as a 7-scalar struct: 3 translation components plus a 4-component unit quaternion.
* Combined transformations can be efficiently applied to points in 3D without calling trigonometric functions.

See the [API reference](API.md) for types, functions and composition semantics.

## Building

This version targets DuckDB **v2.0.0**.

Build (recommended):

```sh
git submodule update --init --recursive
GEN=ninja make
```

The main binaries that will be built are:

```sh
./build/release/duckdb
./build/release/test/unittest
./build/release/extension/se3/se3.duckdb_extension
```

- `duckdb` is the binary for the duckdb shell with the extension code automatically loaded.
- `unittest` is the test runner of duckdb. Again, the extension is already linked into the binary.
- `se3.duckdb_extension` is the loadable binary as it would be distributed.

## Running the extension

To run the extension code, simply start the shell with `./build/release/duckdb`.

Now we can use the features from the extension directly in DuckDB. Example (translate, then rotate):

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

Expected result (first translate the point by (1,0,0) to (2,0,0), then rotate 90° around Z):

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

Expected result (roundoff omitted):

```
{x: -3.0, y: 0.0, z: 0.0}
```

## Running the tests

```sh
make test
```
