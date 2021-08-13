#!/bin/bash

set -e

mkdir -p ../.conan/
conan profile new default --detect
