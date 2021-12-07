#!/bin/bash

set -e

bash init_conan.sh

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../" &>/dev/null && pwd)"
RELEASE_DIR="${PROJECT_DIR}/release"

mkdir -p ${RELEASE_DIR}
cd ${RELEASE_DIR}
conan install .. --build=missing

# config
cmake .. -DCMAKE_BUILD_TYPE=Release

# cmake --build build
# cmake --build build --target myexample
cmake --build . --target routing_server
