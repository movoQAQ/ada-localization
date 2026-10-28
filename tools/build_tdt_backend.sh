#!/usr/bin/env bash
set -euo pipefail
# Run inside the prepared Ubuntu/Jazzy container. Source ROS before invoking.
cd /workspace/z-localization
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /workspace/adaptive-lio-env/backend-log build \
  --build-base /workspace/adaptive-lio-env/backend-build \
  --install-base /workspace/adaptive-lio-env/backend-install \
  --packages-select localization_interfaces backend \
  --cmake-args -DBUILD_TESTING=ON -DNANOFLANN_INCLUDE_DIR=/workspace/adaptive-lio-env/nanoflann-1.4.3
ctest --test-dir /workspace/adaptive-lio-env/backend-build/backend \
  -R '^backend_(descriptor|relocalization|localization|graph|measurement|lifecycle)_test$' --output-on-failure
