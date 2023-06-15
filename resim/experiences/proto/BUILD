load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "revision_proto",
    srcs = ["revision.proto"],
)

cc_proto_library(
    name = "revision_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":revision_proto"],
)

proto_library(
    name = "actor_proto",
    srcs = ["actor.proto"],
    deps = [
        "//resim/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "actor_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":actor_proto"],
)

proto_library(
    name = "location_condition_proto",
    srcs = ["location_condition.proto"],
    deps = [
        ":actor_proto",
        "//resim/transforms/proto:se3_proto",
    ],
)

cc_proto_library(
    name = "location_condition_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":location_condition_proto"],
)

proto_library(
    name = "completion_criteria_proto",
    srcs = ["completion_criteria.proto"],
    deps = [
        ":location_condition_proto",
        "@com_google_protobuf//:duration_proto",
    ],
)

cc_proto_library(
    name = "completion_criteria_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":completion_criteria_proto"],
)

proto_library(
    name = "ilqr_drone_proto",
    srcs = ["ilqr_drone.proto"],
)

cc_proto_library(
    name = "ilqr_drone_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":ilqr_drone_proto"],
)

proto_library(
    name = "storyboard_proto",
    srcs = ["storyboard.proto"],
    deps = [
        ":actor_proto",
        ":ilqr_drone_proto",
        "//resim/actor/state/proto:trajectory_proto",
    ],
)

cc_proto_library(
    name = "storyboard_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":storyboard_proto"],
)

proto_library(
    name = "dynamic_behavior_proto",
    srcs = ["dynamic_behavior.proto"],
    deps = [
        ":actor_proto",
        ":completion_criteria_proto",
        ":storyboard_proto",
    ],
)

cc_proto_library(
    name = "dynamic_behavior_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":dynamic_behavior_proto"],
)

proto_library(
    name = "experience_proto",
    srcs = ["experience.proto"],
    deps = [
        ":dynamic_behavior_proto",
        ":geometry_proto",
        ":revision_proto",
    ],
)

cc_proto_library(
    name = "experience_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":experience_proto"],
)

cc_library(
    name = "experiences_test_helpers",
    testonly = 1,
    srcs = ["experiences_test_helpers.cc"],
    hdrs = ["experiences_test_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor/state:trajectory",
        "//resim/curves:test_helpers",
        "//resim/experiences:actor",
        "//resim/experiences:completion_criteria",
        "//resim/experiences:dynamic_behavior",
        "//resim/experiences:experience",
        "//resim/experiences:geometry",
        "//resim/experiences:ilqr_drone",
        "//resim/experiences:location_condition",
        "//resim/experiences:storyboard",
        "//resim/geometry:drone_wireframe",
        "//resim/geometry:wireframe",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "//resim/utils:match",
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "experiences_test_helpers_test",
    srcs = ["experiences_test_helpers_test.cc"],
    deps = [
        ":experiences_test_helpers",
        "//resim/curves:test_helpers",
        "//resim/experiences:actor",
        "//resim/experiences:completion_criteria",
        "//resim/experiences:dynamic_behavior",
        "//resim/experiences:experience",
        "//resim/experiences:geometry",
        "//resim/experiences:ilqr_drone",
        "//resim/experiences:location_condition",
        "//resim/experiences:storyboard",
        "//resim/time:timestamp",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "actor_to_proto",
    srcs = ["actor_to_proto.cc"],
    hdrs = ["actor_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor_proto_cc",
        "//resim/assert",
        "//resim/experiences:actor",
        "//resim/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "actor_to_proto_test",
    srcs = ["actor_to_proto_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor_proto_cc",
        ":actor_to_proto",
        ":experiences_test_helpers",
        "//resim/assert",
        "//resim/experiences:actor",
        "//resim/utils/proto:uuid_to_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "completion_criteria_to_proto",
    srcs = ["completion_criteria_to_proto.cc"],
    hdrs = ["completion_criteria_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "location_condition_to_proto",
        ":completion_criteria_proto_cc",
        "//resim/assert",
        "//resim/experiences:completion_criteria",
        "//resim/time:timestamp",
    ],
)

cc_test(
    name = "completion_criteria_to_proto_test",
    srcs = ["completion_criteria_to_proto_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":completion_criteria_proto_cc",
        ":completion_criteria_to_proto",
        ":experiences_test_helpers",
        "//resim/assert",
        "//resim/experiences:completion_criteria",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ilqr_drone_to_proto",
    srcs = ["ilqr_drone_to_proto.cc"],
    hdrs = ["ilqr_drone_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":ilqr_drone_proto_cc",
        "//resim/assert",
        "//resim/experiences:ilqr_drone",
    ],
)

cc_test(
    name = "ilqr_drone_to_proto_test",
    srcs = ["ilqr_drone_to_proto_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":experiences_test_helpers",
        ":ilqr_drone_proto_cc",
        ":ilqr_drone_to_proto",
        "//resim/assert",
        "//resim/experiences:ilqr_drone",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "location_condition_to_proto",
    srcs = ["location_condition_to_proto.cc"],
    hdrs = ["location_condition_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor_proto_cc",
        ":location_condition_proto_cc",
        "//resim/assert",
        "//resim/experiences:location_condition",
        "//resim/transforms/proto:se3_to_proto",
        "//resim/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "location_condition_to_proto_test",
    srcs = ["location_condition_to_proto_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":experiences_test_helpers",
        ":location_condition_proto_cc",
        ":location_condition_to_proto",
        "//resim/assert",
        "//resim/experiences:location_condition",
        "//resim/transforms/proto:se3_to_proto",
        "//resim/utils/proto:uuid_to_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "storyboard_to_proto",
    srcs = ["storyboard_to_proto.cc"],
    hdrs = ["storyboard_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor_proto_cc",
        ":actor_to_proto",
        ":ilqr_drone_to_proto",
        ":storyboard_proto_cc",
        "//resim/actor/state/proto:trajectory_to_proto",
        "//resim/assert",
        "//resim/experiences:storyboard",
        "//resim/utils:match",
        "//resim/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "storyboard_to_proto_test",
    srcs = ["storyboard_to_proto_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":experiences_test_helpers",
        ":ilqr_drone_to_proto",
        ":storyboard_proto_cc",
        ":storyboard_to_proto",
        "//resim/actor/state/proto:trajectory_to_proto",
        "//resim/assert",
        "//resim/experiences:storyboard",
        "//resim/utils:match",
        "//resim/utils/proto:uuid_to_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "dynamic_behavior_to_proto",
    srcs = ["dynamic_behavior_to_proto.cc"],
    hdrs = ["dynamic_behavior_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor_proto_cc",
        ":actor_to_proto",
        ":completion_criteria_to_proto",
        ":dynamic_behavior_proto_cc",
        ":storyboard_to_proto",
        "//resim/assert",
        "//resim/experiences:actor",
        "//resim/experiences:dynamic_behavior",
    ],
)

cc_test(
    name = "dynamic_behavior_to_proto_test",
    srcs = ["dynamic_behavior_to_proto_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor_proto_cc",
        ":actor_to_proto",
        ":completion_criteria_to_proto",
        ":dynamic_behavior_proto_cc",
        ":dynamic_behavior_to_proto",
        ":experiences_test_helpers",
        ":storyboard_to_proto",
        "//resim/assert",
        "//resim/experiences:actor",
        "//resim/experiences:dynamic_behavior",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "experience_to_proto",
    srcs = ["experience_to_proto.cc"],
    hdrs = ["experience_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamic_behavior_to_proto",
        ":experience_proto_cc",
        ":geometry_to_proto",
        "//resim/assert",
        "//resim/experiences:experience",
    ],
)

cc_test(
    name = "experience_to_proto_test",
    srcs = ["experience_to_proto_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamic_behavior_to_proto",
        ":experience_proto_cc",
        ":experience_to_proto",
        ":experiences_test_helpers",
        ":geometry_to_proto",
        "//resim/assert",
        "//resim/experiences:experience",
        "//resim/experiences:geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "geometry_proto",
    srcs = ["geometry.proto"],
    deps = [
        "//resim/geometry/proto:wireframe_proto",
        "//resim/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "geometry_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":geometry_proto"],
)

cc_library(
    name = "geometry_to_proto",
    srcs = ["geometry_to_proto.cc"],
    hdrs = ["geometry_to_proto.hh"],
    deps = [
        ":geometry_proto_cc",
        "//resim/assert",
        "//resim/experiences:geometry",
        "//resim/geometry:wireframe",
        "//resim/geometry/proto:wireframe_to_proto",
        "//resim/utils:match",
        "//resim/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "geometry_to_proto_test",
    srcs = ["geometry_to_proto_test.cc"],
    deps = [
        ":geometry_proto_cc",
        ":geometry_to_proto",
        "//resim/assert",
        "//resim/experiences:geometry",
        "//resim/geometry:drone_wireframe",
        "//resim/geometry:wireframe",
        "//resim/geometry/proto:wireframe_to_proto",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)
