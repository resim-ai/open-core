load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:zip.bzl", "pkg_zip")

cc_library(
    name = "parse_view_updates",
    srcs = ["parse_view_updates.cc"],
    hdrs = ["parse_view_updates.hh"],
    deps = [
        "//resim/assert",
        "//resim/visualization:view_primitive",
        "//resim/visualization/proto:view_update_proto_cc",
        "//resim/visualization/proto:view_update_to_proto",
    ],
)

cc_test(
    name = "parse_view_updates_test",
    srcs = ["parse_view_updates_test.cc"],
    deps = [
        ":parse_view_updates",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/utils:uuid",
        "//resim/visualization:view_primitive",
        "//resim/visualization:view_update",
        "//resim/visualization/proto:view_update_proto_cc",
        "//resim/visualization/proto:view_update_to_proto",
        "//resim/visualization/testing:test_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "view_primitives_to_foxglove",
    srcs = ["view_primitives_to_foxglove.cc"],
    hdrs = ["view_primitives_to_foxglove.hh"],
    deps = [
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/time:sample_interval",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:match",
        "//resim/visualization:color",
        "//resim/visualization:view_primitive",
        "//resim/visualization/curve:control_points_from_curve",
        "//resim/visualization/curve:esc_frames_from_t_curve",
        "//resim/visualization/curve:line_primitive_from_d_curve",
        "//resim/visualization/curve:line_primitive_from_t_curve",
        "//resim/visualization/curve:trajectory_from_t_curve",
        "//resim/visualization/curve:visualize_t_curve",
        "//resim/visualization/foxglove:color_to_foxglove",
        "//resim/visualization/foxglove:orientation_to_foxglove",
        "//resim/visualization/foxglove:pose_to_foxglove",
        "//resim/visualization/foxglove:vector_to_foxglove",
        "//resim/visualization/proto:view_primitive_to_metadata_proto",
        "//resim/visualization/transform:arrow_primitive_options",
        "//resim/visualization/transform:orientation_between_vector",
        "@com_google_protobuf//:protobuf",
        "@fmt",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "view_primitives_to_foxglove_test",
    srcs = ["view_primitives_to_foxglove_test.cc"],
    deps = [
        ":view_primitives_to_foxglove",
        ":view_server_test_helper",
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve_test_helpers",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/testing:random_matrix",
        "//resim/testing:test_directory",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/utils:match",
        "//resim/utils:uuid",
        "//resim/visualization:view_primitive",
        "//resim/visualization/curve:line_primitive_from_d_curve",
        "//resim/visualization/curve:line_primitive_from_t_curve",
        "//resim/visualization/curve:trajectory_from_t_curve",
        "//resim/visualization/foxglove:orientation_to_foxglove",
        "//resim/visualization/foxglove:vector_to_foxglove",
        "//resim/visualization/transform:arrow_primitive_options",
        "//resim/visualization/transform:orientation_between_vector",
        "@com_google_googletest//:gtest_main",
        "@fmt",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "view_server_test_helper",
    testonly = 1,
    srcs = ["view_server_test_helper.cc"],
    hdrs = ["view_server_test_helper.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:d_curve_test_helpers",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/testing:random_matrix",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
    ],
)

cc_test(
    name = "view_server_test_helper_test",
    testonly = 1,
    srcs = ["view_server_test_helper_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_server_test_helper",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/curves:d_curve",
        "//resim/curves:d_curve_test_helpers",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/transforms:frame",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "log_primitives",
    srcs = ["log_primitives.cc"],
    hdrs = ["log_primitives.hh"],
    deps = [
        ":view_primitives_to_foxglove",
        "//resim/utils:mcap_logger",
        "//resim/visualization/proto:view_primitive_to_metadata_proto",
    ],
)

cc_test(
    name = "log_primitives_test",
    srcs = ["log_primitives_test.cc"],
    deps = [
        ":log_primitives",
        ":view_server_test_helper",
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/testing:test_directory",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@fmt",
        "@foxglove_schemas",
        "@mcap",
    ],
)

cc_library(
    name = "view_write_log",
    srcs = ["view_write_log.cc"],
    hdrs = [
        "view_write_log.h",
        "view_write_log.hh",
    ],
    deps = [
        ":log_primitives",
        ":parse_view_updates",
        ":view_primitives_to_foxglove",
    ],
)

cc_test(
    name = "view_write_log_test",
    srcs = ["view_write_log_test.cc"],
    deps = [
        ":view_server_test_helper",
        ":view_write_log",
        ":view_write_log_testing",
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/visualization:view_update",
        "//resim/visualization/proto:view_update_proto_cc",
        "//resim/visualization/proto:view_update_to_proto",
        "//resim/visualization/view_server:log_primitives",
        "//resim/visualization/view_server:parse_view_updates",
        "//resim/visualization/view_server:view_primitives_to_foxglove",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "view_write_log_testing",
    srcs = ["view_write_log_testing.cc"],
    hdrs = ["view_write_log_testing.h"],
    deps = [
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/visualization:view_update",
        "//resim/visualization/proto:view_update_proto_cc",
        "//resim/visualization/proto:view_update_to_proto",
        "//resim/visualization/view_server:view_write_log",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "view_write_log_testing_test",
    srcs = ["view_write_log_testing_test.cc"],
    deps = [
        ":view_write_log",
        ":view_write_log_testing",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "resim_view_server",
    srcs = [
        "view_write_log.cc",
        "view_write_log_testing.cc",
    ],
    linkshared = True,
    deps = [
        ":view_write_log",
        ":view_write_log_testing",
    ],
)

filegroup(
    name = "resim_view_write_log_files",
    srcs = [
        "view_write_log.h",
        "view_write_log_testing.h",
        ":resim_view_server",
    ],
)

pkg_zip(
    name = "resim_view_write_log_pkg",
    srcs = [":resim_view_write_log_files"],
)
