#!/usr/bin/env bash
# Web UI test: builds the firmware's protocol parser for the host, starts a
# mock robot serving the real page, and drives it with Playwright.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
${CXX:-g++} -std=c++17 -O1 -Wall -Wextra -Werror -I ../../lib/RobotCore/src \
  ../../lib/RobotCore/src/ControlProtocol.cpp parse_cli.cpp -o build/parse_cli
node ui_test.js
