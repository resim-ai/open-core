# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "auth_messages_proto",
    srcs = ["auth_messages.proto"],
    deps = [
        "//resim/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "auth_messages_proto_cc",
    visibility = ["//resim/auth:__subpackages__"],
    deps = [":auth_messages_proto"],
)
