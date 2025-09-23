# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_oci//oci:defs.bzl", "oci_image", "oci_push", "oci_tarball")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@rules_resim//:defs.bzl", "resim_build")
load("@resim_version//:defs.bzl", version = "RESIM_VERSION", branch = "RESIM_BRANCH")

cc_library(
    name = "step_executor",
    hdrs = ["step_executor.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:uuid",
    ],
)

cc_library(
    name = "simple_step_executor",
    srcs = ["simple_step_executor.cc"],
    hdrs = ["simple_step_executor.hh"],
    visibility = ["//resim/simulator:__subpackages__"],
    deps = [
        ":step_executor",
    ],
)

cc_test(
    name = "simple_step_executor_test",
    srcs = ["simple_step_executor_test.cc"],
    deps = [
        ":simple_step_executor",
        "//resim/utils:inout",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "channel",
    hdrs = ["channel.hh"],
    visibility = ["//resim/simulator:__subpackages__"],
    deps = [
    ],
)

cc_test(
    name = "channel_test",
    srcs = ["channel_test.cc"],
    deps = [
        ":channel",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "channel_registry",
    srcs = ["channel_registry.cc"],
    hdrs = ["channel_registry.hh"],
    visibility = ["//resim/simulator:__subpackages__"],
    deps = [
        ":channel",
        "//resim/assert",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "channel_registry_test",
    srcs = ["channel_registry_test.cc"],
    deps = [
        ":channel",
        ":channel_registry",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "executor_builder",
    srcs = ["executor_builder.cc"],
    hdrs = ["executor_builder.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":channel_registry",
        ":simple_step_executor",
        ":step_executor",
        "//resim/assert",
        "//resim/utils:tuple_utils",
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "executor_builder_test",
    srcs = ["executor_builder_test.cc"],
    deps = [
        ":executor_builder",
        ":simple_step_executor",
        ":step_executor",
        "//resim/assert",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "standard_topics",
    hdrs = ["standard_topics.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_library(
    name = "simulation_unit",
    hdrs = ["simulation_unit.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:mcap_logger",
    ],
)

cc_library(
    name = "time_lord_update",
    hdrs = ["time_lord_update.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/time:timestamp",
    ],
)

cc_test(
    name = "time_lord_update_test",
    srcs = ["time_lord_update_test.cc"],
    deps = [
        ":time_lord_update",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "time_lord",
    srcs = ["time_lord.cc"],
    hdrs = ["time_lord.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":executor_builder",
        ":simulation_unit",
        ":standard_topics",
        ":time_lord_update",
        "//resim/assert",
        "//resim/time:timestamp",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "time_lord_test",
    srcs = ["time_lord_test.cc"],
    deps = [
        ":standard_topics",
        ":step_executor",
        ":time_lord",
        "//resim/assert",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "standard_frames",
    hdrs = ["standard_frames.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/transforms:frame",
        "//resim/utils:uuid",
    ],
)

cc_library(
    name = "simulate",
    srcs = ["simulate.cc"],
    hdrs = ["simulate.hh"],
    deps = [
        ":executor_builder",
        ":step_executor",
        "//resim/actor:actor_unit",
        "//resim/actor:factory",
        "//resim/actor:location_condition_unit",
        "//resim/experiences:experience",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/experiences/proto:experience_to_proto",
        "//resim/metrics:actor_metrics_unit",
        "//resim/simulator:time_lord",
        "//resim/time:timestamp",
        "//resim/utils:inout",
        "//resim/utils:mcap_logger",
    ],
)

cc_test(
    name = "simulate_test",
    srcs = ["simulate_test.cc"],
    deps = [
        ":simulate",
        ":standard_frames",
        "//resim/actor:actor_id",
        "//resim/actor/state:trajectory",
        "//resim/curves:test_helpers",
        "//resim/experiences:actor",
        "//resim/experiences:completion_criteria",
        "//resim/experiences:dynamic_behavior",
        "//resim/experiences:experience",
        "//resim/experiences:storyboard",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/experiences/proto:experience_to_proto",
        "//resim/testing:test_directory",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "@com_google_googletest//:gtest_main",
        "@fmt",
        "@mcap",
    ],
)

cc_binary(
    name = "resim_run",
    srcs = ["resim_run.cc"],
    deps = [
        ":simulate",
        "//resim/assert",
        "//resim/experiences:actor",
        "//resim/experiences:experience",
        "//resim/experiences:ilqr_drone",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/experiences/proto:experience_to_proto",
        "//resim/utils:inout",
        "//resim/utils:uuid",
        "@com_google_protobuf//:protobuf",
        "@cxxopts",
        "@indicators",
    ],
)

sh_binary(
    name = "sim_container_entrypoint",
    srcs = ["sim_container_entrypoint.sh"],
    data = [
        ":resim_run",
        "//resim/visualization/log:make_visualization_log",
    ],
)

pkg_tar(
    name = "entrypoint_layer",
    srcs = [
        ":sim_container_entrypoint",
    ],
    include_runfiles = True,
    strip_prefix = ".",
)

oci_image(
    name = "drone-simulator",
    base = "@debian_slim",
    entrypoint = ["resim/simulator/sim_container_entrypoint"],
    tars = [":entrypoint_layer"],
    workdir = "/sim_container_entrypoint.runfiles/_main",
)

oci_tarball(
    name = "sim_build_tarball",
    image = ":drone-simulator",
    repo_tags = ["drone-simulator:latest"],
)

oci_push(
    name = "sim_build_push",
    image = ":drone-simulator",
    repository = "909785973729.dkr.ecr.us-east-1.amazonaws.com/core-sim-run",
    remote_tags = [version]
)


resim_build(
    name = "drone_simulator_build",
    project = "A ReSim Drone Sandbox",
    system = "Drone Motion Planning System",
    version = version,
    branch = branch,
    description = "An iLQR drone simulator",
    resim_name = "Drone Simulator",
    image_pushes = [
        ":sim_build_push",
    ],
)
