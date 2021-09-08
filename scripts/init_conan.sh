#!/bin/bash

set -e

mkdir -p ${CONAN_USER_HOME}/.conan/
conan profile new default --detect
conan profile update settings.compiler.libcxx=libstdc++11 default
conan profile update settings.compiler.cppstd=gnu17 default
# username: repo, password: dev_tsingroc
conan remote add tsingroc https://conan.tsingroc.com
conan user -p dev_tsingroc repo -r tsingroc
