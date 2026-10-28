#!/usr/bin/env bash
# Backend-only convenience command; dependencies are installed manually.
set -euo pipefail
ADA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
: "${NANOFLANN_INCLUDE_DIR:?Set NANOFLANN_INCLUDE_DIR to the nanoflann 1.4.3 include directory}"
test -f "$NANOFLANN_INCLUDE_DIR/nanoflann.hpp"
cd "$ADA_ROOT"
MAKEFLAGS="${MAKEFLAGS:--j2 -l2}" CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" colcon build \
  --executor sequential --packages-up-to backend \
  --cmake-args -DBUILD_TESTING=ON "-DNANOFLANN_INCLUDE_DIR=$NANOFLANN_INCLUDE_DIR" "$@"
ctest --test-dir build/backend -R '^backend_(descriptor|relocalization|localization|graph|measurement|lifecycle)_test$' --output-on-failure
