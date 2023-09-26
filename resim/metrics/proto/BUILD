# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_python//python:proto.bzl", "py_proto_library")

proto_library(
    name = "simple_metric_proto",
    srcs = ["simple_metric.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils/proto:uuid_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "simple_metric_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":simple_metric_proto"],
)

cc_library(
    name = "simple_metric_to_proto",
    srcs = ["simple_metric_to_proto.cc"],
    hdrs = ["simple_metric_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":simple_metric_proto_cc",
        "//resim/actor:actor_id",
        "//resim/assert",
        "//resim/metrics:simple_metric",
        "//resim/time/proto:time_to_proto",
        "//resim/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "simple_metric_to_proto_test",
    srcs = ["simple_metric_to_proto_test.cc"],
    deps = [
        ":simple_metric_proto_cc",
        "//resim/metrics:simple_metric",
        "//resim/metrics/proto:simple_metric_to_proto",
        "//resim/time:random_duration",
        "//resim/time/proto:time_to_proto",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "metrics_proto",
    srcs = ["metrics.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils/proto:uuid_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "metrics_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":metrics_proto"],
)

py_proto_library(
    name = "metrics_proto_py",
    visibility = ["//visibility:public"],
    deps = [":metrics_proto"],
)
