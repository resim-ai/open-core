# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "view_primitive_proto",
    srcs = ["view_primitive.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor/state/proto:trajectory_proto",
        "//resim/curves/proto:d_curve_proto",
        "//resim/curves/proto:t_curve_proto",
        "//resim/transforms/proto:frame_3_proto",
        "//resim/transforms/proto:framed_vector_3_proto",
        "//resim/transforms/proto:se3_proto",
        "//resim/transforms/proto:so3_proto",
        "//resim/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "view_primitive_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":view_primitive_proto",
    ],
)

cc_library(
    name = "view_primitive_to_proto",
    srcs = ["view_primitive_to_proto.cc"],
    hdrs = ["view_primitive_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_primitive_proto_cc",
        "//resim/actor/state:trajectory",
        "//resim/actor/state/proto:trajectory_to_proto",
        "//resim/assert",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/curves/proto:d_curve_se3_to_proto",
        "//resim/curves/proto:t_curve_se3_to_proto",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/transforms/proto:frame_3_to_proto",
        "//resim/transforms/proto:framed_vector_3_to_proto",
        "//resim/transforms/proto:se3_to_proto",
        "//resim/transforms/proto:so3_to_proto",
        "//resim/utils:match",
        "//resim/utils:status_value",
        "//resim/utils/proto:uuid_to_proto",
        "//resim/visualization:view_primitive",
    ],
)

cc_test(
    name = "view_primitive_to_proto_test",
    srcs = ["view_primitive_to_proto_test.cc"],
    deps = [
        ":view_primitive_test_helper",
        "//resim/actor/state:trajectory",
        "//resim/actor/state/proto:trajectory_to_proto",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/curves/proto:d_curve_se3_to_proto",
        "//resim/curves/proto:t_curve_se3_to_proto",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/transforms/proto:frame_3_to_proto",
        "//resim/transforms/proto:framed_vector_3_to_proto",
        "//resim/transforms/proto:se3_to_proto",
        "//resim/transforms/proto:so3_to_proto",
        "//resim/utils:match",
        "//resim/utils:uuid",
        "//resim/utils/proto:uuid_proto_cc",
        "//resim/utils/proto:uuid_to_proto",
        "//resim/visualization:view_primitive",
        "//resim/visualization/proto:view_primitive_proto_cc",
        "//resim/visualization/proto:view_primitive_to_proto",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

proto_library(
    name = "view_update_proto",
    srcs = ["view_update.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_primitive_proto",
        "//resim/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "view_update_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":view_update_proto",
    ],
)

cc_library(
    name = "view_update_to_proto",
    srcs = ["view_update_to_proto.cc"],
    hdrs = ["view_update_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_primitive_to_proto",
        "//resim/assert",
        "//resim/visualization:view_update",
        "//resim/visualization/proto:view_update_proto_cc",
    ],
)

cc_test(
    name = "view_update_to_proto_test",
    srcs = ["view_update_to_proto_test.cc"],
    deps = [
        ":view_primitive_to_proto",
        ":view_update_to_proto",
        "//resim/assert",
        "//resim/transforms:se3",
        "//resim/utils:match",
        "//resim/utils:uuid",
        "//resim/visualization:view_primitive",
        "//resim/visualization/testing:test_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "view_object_metadata_proto",
    srcs = ["view_object_metadata.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "view_object_metadata_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":view_object_metadata_proto",
    ],
)

cc_library(
    name = "view_primitive_to_metadata_proto",
    srcs = ["view_primitive_to_metadata_proto.cc"],
    hdrs = ["view_primitive_to_metadata_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_object_metadata_proto_cc",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:match",
        "//resim/visualization:view_primitive",
    ],
)

cc_test(
    name = "view_primitive_to_metadata_proto_test",
    srcs = ["view_primitive_to_metadata_proto_test.cc"],
    deps = [
        ":view_primitive_test_helper",
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:match",
        "//resim/visualization:view_primitive",
        "//resim/visualization/proto:view_object_metadata_proto_cc",
        "//resim/visualization/proto:view_primitive_to_metadata_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "view_primitive_test_helper",
    testonly = 1,
    srcs = ["view_primitive_test_helper.cc"],
    hdrs = ["view_primitive_test_helper.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/testing:random_matrix",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view_primitive",
    ],
)

cc_test(
    name = "view_primitive_test_helper_test",
    srcs = ["view_primitive_test_helper_test.cc"],
    deps = [
        ":view_primitive_test_helper",
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:match",
        "//resim/visualization:view_primitive",
        "@com_google_googletest//:gtest_main",
    ],
)
