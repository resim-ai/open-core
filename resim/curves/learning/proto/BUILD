# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "t_curve_distribution_proto",
    srcs = ["t_curve_distribution.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/curves/proto:t_curve_proto",
    ],
)

cc_proto_library(
    name = "t_curve_distribution_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":t_curve_distribution_proto"],
)
