#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname ${SCRIPT_DIR})"

mkdir -p ${PROJECT_DIR}/.conan/
conan profile new default --detect
conan profile update settings.compiler.libcxx=libstdc++11 default
conan profile update settings.compiler.cppstd=gnu17 default
# username: root, password: conan_for_city
conan remote add rl1 http://rl1.cityzoom.cn:9300
