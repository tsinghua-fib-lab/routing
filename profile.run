#!/bin/bash

set -e

./build/bin/routing_profile --flagfile flags/profile_beijing3.flag
google-pprof --pdf ./build/bin/routing_profile profile.log > profile.pdf
