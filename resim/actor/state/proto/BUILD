# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
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
        "//resim/time/proto:time_to_proto",
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

proto_library(
    name = "observable_state_proto",
    srcs = ["observable_state.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/curves/proto:two_jet_proto",
        "//resim/utils/proto:uuid_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "observable_state_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":observable_state_proto"],
)

py_proto_library(
    name = "observable_state_proto_py",
    visibility = ["//visibility:public"],
    deps = [":observable_state_proto"],
)

cc_library(
    name = "observable_state_to_proto",
    srcs = ["observable_state_to_proto.cc"],
    hdrs = ["observable_state_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":observable_state_proto_cc",
        "//resim/actor/state:observable_state",
        "//resim/actor/state:rigid_body_state",
        "//resim/assert",
        "//resim/curves/proto:two_jetr_se3_to_proto",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:se3",
        "//resim/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "observable_state_to_proto_test",
    srcs = ["observable_state_to_proto_test.cc"],
    deps = [
        ":observable_state_proto_cc",
        ":observable_state_to_proto",
        "//resim/actor:actor_id",
        "//resim/actor/state:observable_state",
        "//resim/actor/state:rigid_body_state",
        "//resim/assert",
        "//resim/curves:two_jet",
        "//resim/testing:random_matrix",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)
