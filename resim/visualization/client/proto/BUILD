# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "view_client_proto",
    srcs = ["view_client.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "view_client_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":view_client_proto",
    ],
)
