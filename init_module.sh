#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_DIR=${SCRIPT_DIR}

echo "Project Directory: ${PROJECT_DIR}"
echo "init go module"

cp -r ${PROJECT_DIR}/submodules/protos/wolong ${PROJECT_DIR}
go mod tidy
go mod vendor
buf generate --include-imports --path wolong/routing
rm -r ${PROJECT_DIR}/wolong
