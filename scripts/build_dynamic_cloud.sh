#!/usr/bin/env bash
set -eo pipefail
DYNAMIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DYNAMIC_BUILD="$DYNAMIC_PROJECT/build/dynamic_cloud"
mkdir -p "$DYNAMIC_BUILD"
g++ -O3 -std=c++17 -Wall -Wextra -fPIC -shared \
  "$DYNAMIC_PROJECT/scripts/dynamic_cloud_filter.cpp" -loctomap -loctomath \
  -o "$DYNAMIC_BUILD/libdynamic_cloud.so.tmp"
mv "$DYNAMIC_BUILD/libdynamic_cloud.so.tmp" "$DYNAMIC_BUILD/libdynamic_cloud.so"
