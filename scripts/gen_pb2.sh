#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

echo "Project Directory: ${PROJECT_DIR}"
echo "Generate _pb2.py to proto"

protoc --python_out=${PROJECT_DIR}/protos/py \
    -I ${PROJECT_DIR}/submodules/protos \
    ${PROJECT_DIR}/submodules/protos/wolong/geo/v1/geo.proto \
    ${PROJECT_DIR}/submodules/protos/wolong/routing/v1/routing.proto

echo "Generate grpc"

python3 -m grpc_tools.protoc \
    -I ${PROJECT_DIR}/submodules/protos \
    --python_out=${PROJECT_DIR}/protos/py \
    --grpc_python_out=${PROJECT_DIR}/protos/py \
    ${PROJECT_DIR}/submodules/protos/wolong/routing/v1/routing_service.proto
