# Se3

This repository is based on https://github.com/duckdb/extension-template, check it out if you want to build and ship your own DuckDB extension.

---

This extension, Se3, provides SE(3) rigid transformations using unit quaternions, implemented as vectorized DuckDB scalar functions.

* Sequences of transformations can be combined to a 7 DOF struct that represents a combined translation and rotation.
* Combined transformations can be efficiently applied to points in 3D without calling trigonometric functions.


## Building

Build (recommended):
```sh
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

Compose transformations (apply `W1` then `W2`):
```sql
SELECT se3_apply(
  se3_compose(
    se3_from_axis_angle(struct_pack(x:=0.0, y:=1.0, z:=0.0), struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0),
    se3_from_axis_angle(struct_pack(x:=1.0, y:=0.0, z:=0.0), struct_pack(x:=0.0, y:=0.0, z:=1.0), pi()/2.0)
  ),
  struct_pack(x:=1.0, y:=0.0, z:=0.0)
);
```
Expected result:
```
{x: -3.0, y: 0.0, z: 0.0}
```

For the full API and semantics, see `API.md` and `SPEC.md`.

## Running the tests
```sh
make test
```

### Installing the deployed binaries
To install your extension binaries from S3, you will need to do two things. Firstly, DuckDB should be launched with the
`allow_unsigned_extensions` option set to true. How to set this will depend on the client you're using. Some examples:

CLI:
```shell
duckdb -unsigned
```

Python:
```python
con = duckdb.connect(':memory:', config={'allow_unsigned_extensions' : 'true'})
```

NodeJS:
```js
db = new duckdb.Database(':memory:', {"allow_unsigned_extensions": "true"});
```

Secondly, you will need to set the repository endpoint in DuckDB to the HTTP url of your bucket + version of the extension
you want to install. To do this run the following SQL query in DuckDB:
```sql
SET custom_extension_repository='bucket.s3.eu-west-1.amazonaws.com/<your_extension_name>/latest';
```
Note that the `/latest` path will allow you to install the latest extension version available for your current version of
DuckDB. To specify a specific version, you can pass the version instead.

After running these steps, you can install and load your extension using the regular INSTALL/LOAD commands in DuckDB:
```sql
INSTALL se3;
LOAD se3;
```
