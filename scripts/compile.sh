#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../" &>/dev/null && pwd)"
if [ -d "${CONAN_USER_HOME}/.conan/" ]
then
  echo "Conan directory is existed. Skip conan."
else
  bash init_conan.sh
fi

cd ${PROJECT_DIR}/build/
conan install .. --build=missing
cmake ..
make -j
