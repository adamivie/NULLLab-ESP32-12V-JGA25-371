#!/usr/bin/env bash
# Builds and runs the RobotCore unit tests on the host PC (no board needed).
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p host_tests/build
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I lib/RobotCore/src lib/RobotCore/src/*.cpp host_tests/test_core.cpp \
  -o host_tests/build/test_core
./host_tests/build/test_core
