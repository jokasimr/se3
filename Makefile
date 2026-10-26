PROJ_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# Configuration of extension
EXT_NAME=se3
EXT_CONFIG=${PROJ_DIR}extension_config.cmake

OVERRIDE_GIT_DESCRIBE ?= $(shell git -C ${PROJ_DIR}duckdb describe --tags --exact-match --match 'v[0-9]*' 2>/dev/null)

ifeq ($(wildcard ${PROJ_DIR}duckdb/CMakeLists.txt),)
$(error DuckDB sources are missing. Run: git submodule update --init --recursive)
endif
ifeq ($(wildcard ${PROJ_DIR}extension-ci-tools/makefiles/duckdb_extension.Makefile),)
$(error CI tools are missing. Run: git submodule update --init --recursive)
endif

# Include the Makefile from extension-ci-tools
include ${PROJ_DIR}extension-ci-tools/makefiles/duckdb_extension.Makefile
