load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "trajectory_proto",
    srcs = ["trajectory.proto"],
    deps = [
        "//resim_core/curves/proto:t_curve_proto",
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
        "//resim_core/actor/state:trajectory",
        "//resim_core/assert",
        "//resim_core/curves/proto:t_curve_fse3_to_proto",
        "//resim_core/time:timestamp",
        "//resim_core/utils:inout",
    ],
)

cc_test(
    name = "trajectory_to_proto_test",
    srcs = ["trajectory_to_proto_test.cc"],
    deps = [
        ":trajectory_to_proto",
        "//resim_core/actor/state:trajectory",
        "//resim_core/assert",
        "//resim_core/curves:t_curve",
        "//resim_core/curves:t_curve_test_helpers",
        "//resim_core/curves/proto:t_curve_fse3_to_proto",
        "//resim_core/time:random_duration",
        "//resim_core/time:timestamp",
        "@com_google_googletest//:gtest_main",
    ],
)
