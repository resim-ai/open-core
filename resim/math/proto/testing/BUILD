# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "test_matrix_proto",
    testonly = True,
    srcs = ["test_matrix.proto"],
)

cc_proto_library(
    name = "test_matrix_proto_cc",
    testonly = True,
    visibility = ["//resim/math/proto:__pkg__"],
    deps = [":test_matrix_proto"],
)
