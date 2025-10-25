#!/bin/bash
cat <<EOF
STABLE_RESIM_VERSION $(git rev-parse HEAD)
STABLE_RESIM_BRANCH $(git rev-parse --abbrev-ref HEAD)
EOF
