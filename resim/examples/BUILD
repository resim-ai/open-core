# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_binary")

cc_binary(
    name = "assert_and_status",
    srcs = ["assert_and_status.cc"],
    deps = [
        "//resim/assert",
        "//resim/utils:status",
        "//resim/utils:status_value",
    ],
)

cc_binary(
    name = "curves",
    srcs = ["curves.cc"],
    deps = [
        "//resim/assert",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/curves:two_jet",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "trajectory",
    srcs = ["trajectory.cc"],
    deps = [
        "//resim/actor/state:rigid_body_state",
        "//resim/actor/state:trajectory",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
        "@libeigen//:eigen",
    ],
)

cc_binary(
    name = "gaussian",
    srcs = ["gaussian.cc"],
    deps = [
        "//resim/math:multivariate_gaussian",
        "@fmt",
        "@libeigen//:eigen",
    ],
)

cc_binary(
    name = "time",
    srcs = ["time.cc"],
    deps = [
        "//resim/assert",
        "//resim/time:event_schedule",
        "//resim/time:sample_interval",
        "//resim/time:timestamp",
    ],
)

cc_binary(
    name = "liegroups",
    srcs = ["liegroups.cc"],
    deps = [
        "//resim/assert",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "output_parameters",
    srcs = ["output_parameters.cc"],
    deps = [
        "//resim/utils:inout",
        "//resim/utils:nullable_reference",
    ],
)

cc_binary(
    name = "view",
    srcs = ["view.cc"],
    deps = [
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "match",
    srcs = ["match.cc"],
    deps = [
        "//resim/utils:match",
    ],
)

py_binary(
    name = "plotly_metrics",
    srcs = ["plotly_metrics.py"],
    deps = [
        "//resim/metrics/python:metrics_writer",
        "//resim/metrics/python:plotly_helpers",
    ],
)

py_binary(
    name = "auth_example",
    srcs = ["auth_example.py"],
    deps = [
        "//resim/sdk:client",
        "//resim/sdk/auth:device_code_client",
        "//resim/sdk/auth:username_password_client",
    ],
)
