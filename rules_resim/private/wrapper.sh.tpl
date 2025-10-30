#!/usr/bin/env bash
#
# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

set -euo pipefail

VERSION=$(cat "%{VERSION_FILE}")
BRANCH=$(cat "%{BRANCH_FILE}")

%{PUSH_CMDS}

read -r TAG < %{TAGFILE_PATH} || true # When this file is oneline this returns non-zero

IMAGE_URI="%{REPOSITORY}:${TAG}"

# Build the resim command
CMD="%{RESIM_CLI} builds create \
  --name \"%{RESIM_NAME}\" \
  --project \"%{PROJECT}\" \
  --system \"%{SYSTEM}\" \
  --branch \"${BRANCH}\" \
  --version \"${VERSION}\" \
  --image \"${IMAGE_URI}\" \
  --description \"%{DESCRIPTION}\""


if [[ "%{AUTO_CREATE_BRANCH}" == "true" ]]; then
  CMD+=" --auto-create-branch"
fi

CMD+=" $@"

# Execute the command
echo "Running: $CMD"
eval $CMD
