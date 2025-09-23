#!/usr/bin/env bash

%{PUSH_CMDS}

set -euo pipefail

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
