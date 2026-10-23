#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_binary="$(mktemp /tmp/task_1_angles.XXXXXX)"
trap 'rm -f "$test_binary"' EXIT

"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror \
  "$repo_dir/tests/task_1_angles_test.cpp" -o "$test_binary"
"$test_binary"
