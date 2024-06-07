# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "test_message_proto",
    testonly = 1,
    srcs = ["test_message.proto"],
)

cc_proto_library(
    name = "test_message_proto_cc",
    testonly = 1,
    visibility = ["//visibility:public"],
    deps = [":test_message_proto"],
)
