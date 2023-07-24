load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "actor_id",
    hdrs = ["actor_id.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:uuid",
    ],
)

cc_library(
    name = "actor",
    srcs = ["actor.cc"],
    hdrs = ["actor.hh"],
    deps = [
        ":actor_id",
        "//resim/actor/state:observable_state",
        "//resim/time:timestamp",
    ],
)

cc_test(
    name = "actor_test",
    srcs = ["actor_test.cc"],
    deps = [
        ":actor",
        ":actor_id",
        ":test_actor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "test_actor",
    testonly = True,
    srcs = ["test_actor.cc"],
    hdrs = ["test_actor.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor",
        ":actor_id",
        "//resim/actor/state:observable_state",
        "//resim/assert",
        "//resim/simulator:standard_frames",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "@fmt",
    ],
)

cc_library(
    name = "actor_unit",
    srcs = ["actor_unit.cc"],
    hdrs = ["actor_unit.hh"],
    visibility = ["//resim:__subpackages__"],
    deps = [
        ":actor",
        "//resim/actor/state:observable_state",
        "//resim/actor/state/proto:observable_state_proto_cc",
        "//resim/actor/state/proto:observable_state_to_proto",
        "//resim/assert",
        "//resim/simulator:executor_builder",
        "//resim/simulator:simulation_unit",
        "//resim/simulator:standard_frames",
        "//resim/simulator:standard_topics",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "//resim/utils:inout",
        "@fmt",
    ],
)

cc_test(
    name = "actor_unit_test",
    srcs = ["actor_unit_test.cc"],
    deps = [
        ":actor",
        ":actor_id",
        ":actor_unit",
        ":test_actor",
        "//resim/actor/state:observable_state",
        "//resim/actor/state/proto:observable_state_proto_cc",
        "//resim/actor/state/proto:observable_state_to_proto",
        "//resim/simulator:executor_builder",
        "//resim/time:timestamp",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/utils:inout",
        "//resim/utils/testing:mock_logger",
        "@com_google_googletest//:gtest_main",
        "@fmt",
    ],
)

cc_library(
    name = "trajectory_actor",
    srcs = ["trajectory_actor.cc"],
    hdrs = ["trajectory_actor.hh"],
    deps = [
        ":actor",
        ":actor_id",
        "//resim/actor/state:observable_state",
        "//resim/actor/state:trajectory",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "//resim/utils:match",
    ],
)

cc_test(
    name = "trajectory_actor_test",
    srcs = ["trajectory_actor_test.cc"],
    deps = [
        ":actor_id",
        ":trajectory_actor",
        "//resim/actor/state:trajectory",
        "//resim/curves:test_helpers",
        "//resim/simulator:standard_frames",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "factory",
    srcs = ["factory.cc"],
    hdrs = ["factory.hh"],
    visibility = ["//resim:__subpackages__"],
    deps = [
        ":actor_id",
        ":actor_unit",
        ":trajectory_actor",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/experiences:dynamic_behavior",
        "//resim/experiences:storyboard",
        "//resim/utils:match",
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "factory_test",
    srcs = ["factory_test.cc"],
    deps = [
        ":actor",
        ":actor_id",
        ":factory",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/experiences:actor",
        "//resim/experiences:dynamic_behavior",
        "//resim/experiences:storyboard",
        "//resim/simulator:standard_frames",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)
