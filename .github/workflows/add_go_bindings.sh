#!/bin/bash

CURRENT_BRANCH=$(git branch --show-current)

# TODO(michael) Consider replacing some of this with some starklark
# code rather than hard coding everyting

if [[ -z $RESIM_VERSION ]]
then
    echo "RELASE_TAG must be defined"
    exit 1
fi

BAZEL_BIN=$(bazel info bazel-bin)

bazel build //resim/utils/proto:uuid_proto_go
bazel build //resim/metrics/proto:metrics_proto_go

ls ${BAZEL_BIN}/resim/utils/proto
cp -r $BAZEL_BIN/resim/utils/proto/uuid_proto_go_/github.com/resim-ai/open-core/* .
cp -r $BAZEL_BIN/resim/metrics/proto/metrics_proto_go_/github.com/resim-ai/open-core/* .

git checkout -b "prod/go/$RESIM_VERSION"
git add uuid_proto
git add metrics_proto
git commit -m "prod/go/$RESIM_VERSION"
git push --set-upstream origin "prod/go/$RESIM_VERSION" --force

git checkout $CURRENT_BRANCH
