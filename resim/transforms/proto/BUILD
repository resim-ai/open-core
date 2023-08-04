# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

cc_library(
    name = "liegroup_to_proto",
    srcs = ["liegroup_to_proto.cc"],
    hdrs = ["liegroup_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_to_proto",
        ":se3_proto_cc",
        ":so3_proto_cc",
        "//resim/assert",
        "//resim/math/proto:matrix_to_proto",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "liegroup_to_proto_test",
    srcs = ["liegroup_to_proto_test.cc"],
    deps = [
        ":se3_proto_cc",
        ":se3_to_proto",
        ":so3_proto_cc",
        ":so3_to_proto",
        "//resim/assert",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "frame_3_proto",
    srcs = ["frame_3.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "frame_3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":frame_3_proto"],
)

cc_library(
    name = "frame_3_to_proto",
    srcs = ["frame_3_to_proto.cc"],
    hdrs = ["frame_3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_proto_cc",
        "//resim/assert",
        "//resim/transforms:frame",
        "//resim/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "frame_3_to_proto_test",
    srcs = ["frame_3_to_proto_test.cc"],
    deps = [
        ":frame_3_proto_cc",
        ":frame_3_to_proto",
        "//resim/assert",
        "//resim/transforms:frame",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "framed_vector_3_proto",
    srcs = ["framed_vector_3.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_proto",
    ],
)

cc_proto_library(
    name = "framed_vector_3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":framed_vector_3_proto"],
)

cc_library(
    name = "framed_vector_3_to_proto",
    srcs = ["framed_vector_3_to_proto.cc"],
    hdrs = ["framed_vector_3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_to_proto",
        ":framed_vector_3_proto_cc",
        "//resim/assert",
        "//resim/math/proto:matrix_to_proto",
        "//resim/transforms:framed_vector",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "framed_vector_3_to_proto_test",
    srcs = ["framed_vector_3_to_proto_test.cc"],
    deps = [
        ":framed_vector_3_to_proto",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

proto_library(
    name = "so3_proto",
    srcs = ["so3.proto"],
    visibility = ["//visibility:public"],
    deps = [":frame_3_proto"],
)

cc_proto_library(
    name = "so3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":so3_proto"],
)

cc_library(
    name = "so3_to_proto",
    srcs = ["so3_to_proto.cc"],
    hdrs = ["so3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup_to_proto",
        ":so3_proto_cc",
        "//resim/transforms:so3",
    ],
)

proto_library(
    name = "se3_proto",
    srcs = ["se3.proto"],
    visibility = ["//visibility:public"],
    deps = [":frame_3_proto"],
)

cc_proto_library(
    name = "se3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":se3_proto"],
)

cc_library(
    name = "se3_to_proto",
    srcs = ["se3_to_proto.cc"],
    hdrs = ["se3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup_to_proto",
        ":se3_proto_cc",
        "//resim/transforms:se3",
    ],
)
