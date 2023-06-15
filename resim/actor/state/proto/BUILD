load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "trajectory_proto",
    srcs = ["trajectory.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/curves/proto:t_curve_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "trajectory_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":trajectory_proto"],
)

cc_library(
    name = "trajectory_to_proto",
    srcs = ["trajectory_to_proto.cc"],
    hdrs = ["trajectory_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":trajectory_proto_cc",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/curves/proto:t_curve_se3_to_proto",
        "//resim/time:timestamp",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "trajectory_to_proto_test",
    srcs = ["trajectory_to_proto_test.cc"],
    deps = [
        ":trajectory_to_proto",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/curves:t_curve_test_helpers",
        "//resim/curves/proto:t_curve_se3_to_proto",
        "//resim/time:random_duration",
        "//resim/time:timestamp",
        "@com_google_googletest//:gtest_main",
    ],
)
