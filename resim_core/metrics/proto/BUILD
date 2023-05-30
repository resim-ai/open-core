load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "simple_metric_proto",
    srcs = ["simple_metric.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/utils/proto:uuid_proto",
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
        "//resim_core/actor:actor_id",
        "//resim_core/assert",
        "//resim_core/metrics:simple_metric",
        "//resim_core/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "simple_metric_to_proto_test",
    srcs = ["simple_metric_to_proto_test.cc"],
    deps = [
        ":simple_metric_proto_cc",
        "//resim_core/metrics:simple_metric",
        "//resim_core/metrics/proto:simple_metric_to_proto",
        "//resim_core/time:random_duration",
        "//resim_core/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)
