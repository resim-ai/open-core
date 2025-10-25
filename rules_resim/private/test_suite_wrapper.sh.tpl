#!/usr/bin/env bash
#
# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

set -euo pipefail

BUILD_OUTPUT=$(%{CREATE_BUILD_CMD} --github | tail -n 1)
BUILD_ID="${BUILD_OUTPUT#build_id=}"

CMD="%{RESIM_CLI} test-suites run"
CMD+=" --project \"%{PROJECT}\""
CMD+=" --test-suite \"%{TEST_SUITE}\""
CMD+=" --build-id \"${BUILD_ID}\""
CMD+=" %{ADDITIONAL_FLAGS}"
CMD+=" $@"

# Execute the command
echo "Running: $CMD"
eval $CMD
