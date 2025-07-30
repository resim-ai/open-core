# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "wireframe_proto",
    srcs = ["wireframe.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "wireframe_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":wireframe_proto"],
)

cc_library(
    name = "wireframe_to_proto",
    srcs = ["wireframe_to_proto.cc"],
    hdrs = ["wireframe_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":wireframe_proto_cc",
        "//resim/assert",
        "//resim/geometry:wireframe",
        "//resim/math/proto:matrix_to_proto",
    ],
)

cc_test(
    name = "wireframe_to_proto_test",
    srcs = ["wireframe_to_proto_test.cc"],
    deps = [
        ":wireframe_proto_cc",
        ":wireframe_to_proto",
        "//resim/assert",
        "//resim/geometry:wireframe",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "oriented_box_proto",
    srcs = ["oriented_box.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/transforms/proto:se3_proto",
    ],
)

cc_proto_library(
    name = "oriented_box_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":oriented_box_proto"],
)

cc_library(
    name = "fuzz_helpers",
    testonly = True,
    srcs = ["fuzz_helpers.cc"],
    hdrs = ["fuzz_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":oriented_box_proto_cc",
        "//resim/converter:fuzz_helpers",
        "//resim/transforms:se3",
        "//resim/transforms/proto:fuzz_helpers",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "fuzz_helpers_test",
    srcs = ["fuzz_helpers_test.cc"],
    deps = [
        ":fuzz_helpers",
        ":oriented_box_proto_cc",
        "//resim/converter:fuzz_helpers",
        "//resim/transforms/proto:fuzz_helpers",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)
