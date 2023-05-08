load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "step_executor",
    hdrs = ["step_executor.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
    ],
)

cc_library(
    name = "simple_step_executor",
    srcs = ["simple_step_executor.cc"],
    hdrs = ["simple_step_executor.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
        ":step_executor",
    ],
)

cc_test(
    name = "simple_step_executor_test",
    srcs = ["simple_step_executor_test.cc"],
    deps = [
        ":simple_step_executor",
        "//resim_core/utils:inout",
        "//resim_core/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "channel",
    hdrs = ["channel.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
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
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
        ":channel",
        "//resim_core/assert",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "channel_registry_test",
    srcs = ["channel_registry_test.cc"],
    deps = [
        ":channel",
        ":channel_registry",
        "//resim_core/assert",
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
        "//resim_core/assert",
    ],
)

cc_test(
    name = "executor_builder_test",
    srcs = ["executor_builder_test.cc"],
    deps = [
        ":executor_builder",
        ":simple_step_executor",
        ":step_executor",
        "//resim_core/assert",
        "//resim_core/utils:inout",
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
    deps = [],
)

cc_test(
    name = "simulation_unit_test",
    srcs = ["simulation_unit_test.cc"],
    deps = [
        ":simulation_unit",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "time_lord",
    srcs = ["time_lord.cc"],
    hdrs = ["time_lord.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
        ":executor_builder",
        ":simulation_unit",
        ":standard_topics",
        "//resim_core/assert",
        "//resim_core/time:timestamp",
        "//resim_core/utils:inout",
    ],
)

cc_test(
    name = "time_lord_test",
    srcs = ["time_lord_test.cc"],
    deps = [
        ":standard_topics",
        ":step_executor",
        ":time_lord",
        "//resim_core/assert",
        "//resim_core/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "logger_unit",
    srcs = ["logger_unit.cc"],
    hdrs = ["logger_unit.hh"],
    deps = [
        ":standard_frames",
        ":standard_topics",
        "//resim_core/actor:geometry",
        "//resim_core/actor/state:observable_state",
        "//resim_core/assert",
        "//resim_core/simulator:executor_builder",
        "//resim_core/simulator:simulation_unit",
        "//resim_core/time:timestamp",
        "//resim_core/transforms:framed_group",
        "//resim_core/utils:inout",
        "//resim_core/utils:mcap_logger",
        "//resim_core/visualization/foxglove:actor_geometry_to_foxglove",
        "//resim_core/visualization/foxglove:frame_transform_to_foxglove",
        "@fmt",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "logger_unit_test",
    srcs = ["logger_unit_test.cc"],
    deps = [
        ":executor_builder",
        ":logger_unit",
        ":standard_frames",
        ":standard_topics",
        ":step_executor",
        "//resim_core/actor:geometry",
        "//resim_core/actor/state:observable_state",
        "//resim_core/actor/state:rigid_body_state",
        "//resim_core/time:timestamp",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_test_helpers",
        "//resim_core/transforms:se3",
        "//resim_core/utils:inout",
        "//resim_core/utils:mcap_logger",
        "//resim_core/utils:uuid",
        "//resim_core/visualization/foxglove:frame_transform_to_foxglove",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
    ],
)

cc_library(
    name = "standard_frames",
    hdrs = ["standard_frames.hh"],
    visibility = ["//resim_core:__subpackages__"],
    deps = [
        "//resim_core/transforms:frame",
        "//resim_core/utils:uuid",
    ],
)
