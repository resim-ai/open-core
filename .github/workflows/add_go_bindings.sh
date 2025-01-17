#!/bin/bash

set -ex

CURRENT_BRANCH=$(git branch --show-current)

# TODO(michael) Consider replacing some of this with some starklark
# code rather than hard coding everything

if [[ -z $RESIM_VERSION ]]
then
    echo "RELASE_TAG must be defined"
    exit 1
fi

BAZEL_BIN=$(bazel info bazel-bin)

bazel build --remote_download_all //resim/utils/proto:uuid_proto_go
bazel build --remote_download_all //resim/metrics/proto:metrics_proto_go

ls ${BAZEL_BIN}/resim/utils/proto
cp -r $BAZEL_BIN/resim/utils/proto/uuid_proto_go_/github.com/resim-ai/open-core/* .
cp -r $BAZEL_BIN/resim/metrics/proto/metrics_proto_go_/github.com/resim-ai/open-core/* .

git checkout -b "$RESIM_VERSION-go-branch"
git add uuid_proto
git add metrics_proto
git commit -m "Generate go bindings for $RESIM_VERSION-go"
git tag "$RESIM_VERSION-go"
git push origin tag "$RESIM_VERSION-go"

git checkout $CURRENT_BRANCH
