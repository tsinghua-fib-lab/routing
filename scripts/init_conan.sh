#!/bin/bash

set -e

mkdir -p ../.conan/
conan profile new default --detect
conan profile update settings.compiler.libcxx=libstdc++11 default
conan profile update settings.compiler.cppstd=gnu17 default
