# Extension updating

## Current target: DuckDB 2.0

Both submodules and the distribution workflow target `v2.0-cyanoptera`, DuckDB's
2.0 release branch. The submodule commits pin the sources used for local builds;
the CI workflow follows the release branch. Initialize them before building:

```sh
git submodule update --init --recursive
GEN=ninja make
make test
```

The migration from 1.5.4 uses these equivalent C++ API operations:

- Struct field names use `child_list_t<LogicalType>` (now backed by `Identifier`),
  and function-set names use `Identifier`. Field names, order, types, overloads,
  and descriptions are unchanged.
- Scalar signatures now own their parameter names. Registration copies the existing
  description names into each signature so `duckdb_functions()` reports them.
- `StructVector::GetEntries` returns `vector<Vector>` instead of
  `vector<unique_ptr<Vector>>`. Children are accessed directly. Filtered structs
  carry the selections in their children; each child still gets its own unified
  format and selection index.
- Output buffers use `FlatVector::GetDataMutable` and `ValidityMutable`. Vector
  lengths are set explicitly; setting a struct's length propagates recursively
  to its children.
- `ToUnifiedFormat(format)` uses the vector's stored length. The old count-taking
  overload in 2.0 delegates to this method and ignores its count argument.
- `CannotHaveNull()` replaces `AllValid()` with the same validity-mask fast path.
  Per-row checks still apply each input's selection vector, and
  `FlatVector::SetNull` still marks nested struct children NULL recursively.
- `duckdb_extension_statically_link(se3)` preserves the shell and test runner's
  built-in extension. `duckdb_extension_load` alone now only builds it.

The numerical kernels and scalar function NULL-handling defaults are unchanged.
`test/sql/vector_execution.test` supplements the existing API and numerical tests
with filtered structs, constant inputs, nested NULLs, and multiple chunks with a
partial final chunk.

## Updating to subsequent releases

When cloning this template, the target version of DuckDB should be the latest stable release of DuckDB. However, there 
will inevitably come a time when a new DuckDB is released and the extension repository needs updating. This process goes
as follows:

- Bump submodules
  - `./duckdb` should be set to latest tagged release
  - `./extension-ci-tools` should be set to updated branch corresponding to latest DuckDB release. So if you're building for DuckDB `v1.1.0` there will be a branch in `extension-ci-tools` named `v1.1.0` to which you should check out. 
- Bump versions in `.github/workflows`
  - `duckdb_version` input in `duckdb-stable-build` job in `MainDistributionPipeline.yml` should be set to latest tagged release
  - `duckdb_version` input in `duckdb-stable-deploy` job in `MainDistributionPipeline.yml` should be set to latest tagged release
  - the reusable workflow `duckdb/extension-ci-tools/.github/workflows/_extension_distribution.yml` for the `duckdb-stable-build` job should be set to latest tagged release

# API changes
DuckDB extensions built with this extension template are built against the internal C++ API of DuckDB. This API is not guaranteed to be stable.
What this means for extension development is that when updating your extensions DuckDB target version using the above steps, you may run into the fact that your extension no longer builds properly.

Currently, DuckDB does not (yet) provide a specific change log for these API changes, but it is generally not too hard to figure out what has changed.

For figuring out how and why the C++ API changed, we recommend using the following resources:
- DuckDB's [Release Notes](https://github.com/duckdb/duckdb/releases)
- DuckDB's history of [Core extension patches](https://github.com/duckdb/duckdb/commits/main/.github/patches/extensions)
- The git history of the relevant C++ Header file of the API that has changed
