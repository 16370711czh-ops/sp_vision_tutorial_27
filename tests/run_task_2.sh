#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$repo_dir" -B "$repo_dir/build-offline" -DBUILD_VISION_APP=OFF -DBUILD_TESTING=ON
cmake --build "$repo_dir/build-offline" -j2
ctest --test-dir "$repo_dir/build-offline" --output-on-failure
