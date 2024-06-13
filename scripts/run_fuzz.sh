#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

cd "${PROJECT_DIR}"

# 加载.env
set -a # 将后续的变量自动导出
[ -f .env ] && . .env
set +a # 取消自动导出

go test . -v -cover -coverpkg git.fiblab.net/sim/routing/router -fuzz FuzzRouter -fuzztime 600s -parallel 16
