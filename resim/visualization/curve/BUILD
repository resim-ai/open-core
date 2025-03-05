# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "esc_frames_from_t_curve",
    srcs = ["esc_frames_from_t_curve.cc"],
    hdrs = ["esc_frames_from_t_curve.hh"],
    visibility = ["//resim/visualization:__subpackages__"],
    deps = [
        ":poses_in_frame_from_t_curve",
        "//resim/curves:t_curve",
        "//resim/time:sample_interval",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "esc_frames_from_t_curve_test",
    srcs = ["esc_frames_from_t_curve_test.cc"],
    deps = [
        ":esc_frames_from_t_curve",
        "//resim/curves:test_helpers",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:se3",
        "//resim/visualization/foxglove:pose_to_foxglove",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "line_primitive_options",
    hdrs = ["line_primitive_options.hh"],
    deps = [
        "//resim/visualization:color",
    ],
)

cc_library(
    name = "line_primitive_from_t_curve",
    srcs = ["line_primitive_from_t_curve.cc"],
    hdrs = ["line_primitive_from_t_curve.hh"],
    visibility = ["//resim/visualization:__subpackages__"],
    deps = [
        ":line_primitive_options",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/time:sample_interval",
        "//resim/visualization:color",
        "//resim/visualization/foxglove:color_to_foxglove",
        "//resim/visualization/foxglove:pose_to_foxglove",
        "//resim/visualization/foxglove:vector_to_foxglove",
        "@foxglove_schemas",
    ],
)

cc_library(
    name = "line_primitive_from_curve_test_helpers",
    srcs = ["line_primitive_from_curve_test_helpers.cc"],
    hdrs = ["line_primitive_from_curve_test_helpers.hh"],
    visibility = ["//resim/visualization:__subpackages__"],
    deps = [
        "//resim/visualization:color",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "line_primitive_from_t_curve_test",
    srcs = ["line_primitive_from_t_curve_test.cc"],
    deps = [
        ":line_primitive_from_curve_test_helpers",
        ":line_primitive_from_t_curve",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/time:sample_interval",
        "//resim/visualization:color",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "poses_in_frame_from_t_curve",
    srcs = ["poses_in_frame_from_t_curve.cc"],
    hdrs = ["poses_in_frame_from_t_curve.hh"],
    visibility = ["//resim/visualization:__subpackages__"],
    deps = [
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:se3",
        "//resim/visualization/foxglove:pose_to_foxglove",
    ],
)

cc_test(
    name = "poses_in_frame_from_t_curve_test",
    srcs = ["poses_in_frame_from_t_curve_test.cc"],
    deps = [
        ":poses_in_frame_from_t_curve",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "visualize_t_curve",
    srcs = ["visualize_t_curve.cc"],
    hdrs = ["visualize_t_curve.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":esc_frames_from_t_curve",
        ":line_primitive_from_t_curve",
        ":line_primitive_options",
        ":poses_in_frame_from_t_curve",
        "//resim/curves:t_curve",
        "//resim/time:sample_interval",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/utils:inout",
        "//resim/utils:mcap_logger",
    ],
)

cc_test(
    name = "visualize_t_curve_test",
    srcs = ["visualize_t_curve_test.cc"],
    deps = [
        ":visualize_t_curve",
        "//resim/curves:t_curve",
        "//resim/testing:random_matrix",
        "//resim/testing:test_directory",
        "//resim/transforms:se3",
        "//resim/utils:inout",
        "//resim/utils:mcap_logger",
        "@com_google_googletest//:gtest_main",
    ],
)
