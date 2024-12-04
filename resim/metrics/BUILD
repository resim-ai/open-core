# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

cc_library(
    name = "simple_metric",
    hdrs = ["simple_metric.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor:actor_id",
        "//resim/time:timestamp",
    ],
)

cc_library(
    name = "min_distance",
    srcs = ["min_distance.cc"],
    hdrs = ["min_distance.hh"],
    deps = [
        "//resim/actor:actor_id",
        "//resim/actor/state:observable_state",
        "//resim/assert",
        "//resim/transforms:se3",
    ],
)

cc_test(
    name = "min_distance_test",
    srcs = ["min_distance_test.cc"],
    deps = [
        ":min_distance",
        "//resim/actor/state:observable_state",
        "//resim/actor/state:rigid_body_state",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "actor_metrics_unit",
    srcs = ["actor_metrics_unit.cc"],
    hdrs = ["actor_metrics_unit.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor/state:observable_state",
        "//resim/assert",
        "//resim/metrics:min_distance",
        "//resim/metrics:simple_metric",
        "//resim/metrics/proto:simple_metric_proto_cc",
        "//resim/metrics/proto:simple_metric_to_proto",
        "//resim/simulator:executor_builder",
        "//resim/simulator:simulation_unit",
        "//resim/simulator:standard_topics",
        "//resim/time:timestamp",
        "//resim/utils:inout",
        "@fmt",
    ],
)

cc_test(
    name = "actor_metrics_unit_test",
    srcs = ["actor_metrics_unit_test.cc"],
    deps = [
        ":actor_metrics_unit",
        "//resim/actor:test_actor",
        "//resim/actor/state:observable_state",
        "//resim/actor/state:rigid_body_state",
        "//resim/simulator:executor_builder",
        "//resim/simulator:standard_topics",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/utils:inout",
        "//resim/utils:mcap_logger",
        "//resim/utils:uuid",
        "//resim/utils/testing:mock_logger",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "dice_coefficient",
    srcs = ["dice_coefficient.py"],
    visibility = ["//pkg:__subpackages__"],
    deps = [
        "//resim/msg:detection_proto_py",
    ],
)

py_test(
    name = "dice_coefficient_test",
    srcs = ["dice_coefficient_test.py"],
    deps = [
        ":dice_coefficient",
    ],
)

py_library(
    name = "fetch_all_pages",
    srcs = ["fetch_all_pages.py"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

py_test(
    name = "fetch_all_pages_test",
    srcs = ["fetch_all_pages_test.py"],
    deps = [
        ":fetch_all_pages",
    ],
)

py_library(
    name = "fetch_metrics_urls",
    srcs = ["fetch_metrics_urls.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":fetch_all_pages",
        "//resim-python-client",
    ],
)

py_test(
    name = "fetch_metrics_urls_test",
    srcs = ["fetch_metrics_urls_test.py"],
    deps = [
        ":fetch_all_pages",
        ":fetch_metrics_urls",
        "//resim-python-client",
    ],
)

py_library(
    name = "get_metrics_proto",
    srcs = ["get_metrics_proto.py"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_protobuf//:protobuf_python",
        requirement("requests"),
    ],
)

py_test(
    name = "get_metrics_proto_test",
    srcs = ["get_metrics_proto_test.py"],
    deps = [
        ":get_metrics_proto",
        "@com_google_protobuf//:protobuf_python",
    ],
)

py_library(
    name = "fetch_job_metrics",
    srcs = ["fetch_job_metrics.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":fetch_all_pages",
        ":fetch_metrics_urls",
        ":get_metrics_proto",
        "//resim-python-client",
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/metrics/python:unpack_metrics",
    ],
)

py_test(
    name = "fetch_job_metrics_test",
    srcs = ["fetch_job_metrics_test.py"],
    deps = [
        "//resim/metrics/proto:metrics_prot
        ":fetch_job_metrics",
    ],
)

py_library(
    name = "fetch_report_metrics",
    srcs = ["fetch_report_metrics.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":fetch_all_pages",
        "//resim-python-client",
    ],
)

py_library(
    name = "resim_style",
    srcs = ["resim_style.py"],
    visibility = ["//visibility:public"],
    deps = [
        requirement("plotly"),
        "//resim-python-client",
    ],
)

py_test(
    name = "fetch_report_metrics_test",
    srcs = ["fetch_report_metrics_test.py"],
    deps = [
        ":fetch_report_metrics",
        "//resim-python-client",
    ],
)

py_binary(
    name = "default_report_metrics",
    srcs = ["default_report_metrics.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":fetch_report_metrics",
        ":resim_style",
        "//resim/metrics/proto:validate_metrics_proto",
        "//resim/metrics/python:metrics_utils",
        "//resim/metrics/python:metrics_writer",
        requirement("pandas"),
        requirement("httpx"),
        requirement("plotly"),
    ],
)

pkg_tar(
    name = "resim_metrics_tools",
    srcs = [
        ":dice_coefficient",
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/metrics/proto:validate_metrics_proto",
        "//resim/msg:detection_proto_py",
        "//resim/msg:header_proto_py",
        "//resim/msg:navsat_proto_py",
        "//resim/msg:odometry_proto_py",
        "//resim/msg:pose_proto_py",
        "//resim/msg:transform_proto_py",
        "//resim/ros2:convert_log",
        "//resim/transforms/python:se3_python.so",
        "//resim/transforms/python:so3_python.so",
    ],
    extension = "tar.gz",
    include_runfiles = True,
    strip_prefix = "/",
)
