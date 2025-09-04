#!/usr/bin/env bash
set -euo pipefail


echo $(realpath %{TAGFILE_PATH})

%{PUSH_CMDS}


read -r TAG < %{TAGFILE_PATH} || true # When this file is oneline this returns non-zero

IMAGE_URI="%{REPOSITORY}:${TAG}"


# Build the resim command
CMD="%{RESIM_CLI} builds create \
  --name \"%{RESIM_NAME}\" \
  --project \"%{PROJECT}\" \
  --system \"%{SYSTEM}\" \
  --branch \"%{BRANCH}\" \
  --version \"%{VERSION}\" \
  --image \"${IMAGE_URI}\" \
  --description \"%{DESCRIPTION}\""


if [[ "%{AUTO_CREATE_BRANCH}" == "true" ]]; then
  CMD+=" --auto-create-branch"
fi


# Execute the command
echo "Running: $CMD"
eval $CMD
