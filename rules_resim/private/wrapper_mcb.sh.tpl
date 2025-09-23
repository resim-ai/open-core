#!/usr/bin/env bash
#
# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

set -euo pipefail

%{PUSH_CMDS}

# Build the resim command
CMD="%{RESIM_CLI} builds create \
  --name \"%{RESIM_NAME}\" \
  --project \"%{PROJECT}\" \
  --system \"%{SYSTEM}\" \
  --branch \"%{BRANCH}\" \
  --version \"%{VERSION}\" \
  --description \"%{DESCRIPTION}\" \
  --build-spec \"%{BUILD_SPEC}\" \
  --env-files \"%{ENV_FILE_PATH}\""

if [[ "%{AUTO_CREATE_BRANCH}" == "true" ]]; then
  CMD+=" --auto-create-branch"
fi

CMD+=" $@"

# Execute the command
echo "Running: $CMD"
eval $CMD
