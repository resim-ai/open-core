#!/bin/bash


# TODO(michael) Consider replacing this with some starklark code
# rather than hard coding everyting

if [[ -z $RELEASE_TAG ]]
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

git checkout -b "go/$RELEASE_TAG"
git add uuid_proto
git add metrics_proto
git commit -m "go/$RELEASE_TAG"
