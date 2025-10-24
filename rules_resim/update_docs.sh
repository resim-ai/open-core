#!/usr/bin/env bash
#
# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

set -euo pipefail

for file in rules.md extensions.md; do
  cp "$file" "${BUILD_WORKSPACE_DIRECTORY}/$file"
  chmod +w "${BUILD_WORKSPACE_DIRECTORY}/$file"
done
