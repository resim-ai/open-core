#  Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "visit_topic",
    srcs = ["visit_topic.cc"],
    hdrs = ["visit_topic.hh"],
    deps = [
        "//resim/assert",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "visit_topic_test",
    srcs = ["visit_topic_test.cc"],
    deps = [
        ":test_helpers",
        ":visit_topic",
        "//resim/assert",
        "//resim/experiences:experience",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/experiences/proto:experience_to_proto",
        "//resim/third_party/mcap:mcap_impl",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "extract_experience",
    srcs = ["extract_experience.cc"],
    hdrs = ["extract_experience.hh"],
    deps = [
        ":visit_topic",
        "//resim/experiences:experience",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/experiences/proto:experience_to_proto",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "extract_experience_test",
    srcs = ["extract_experience_test.cc"],
    deps = [
        ":extract_experience",
        ":test_helpers",
        "//resim/assert",
        "//resim/experiences:experience",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/experiences/proto:experience_to_proto",
        "//resim/experiences/proto:experiences_test_helpers",
        "//resim/third_party/mcap:mcap_impl",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "visualize_actor_states",
    srcs = ["visualize_actor_states.cc"],
    hdrs = ["visualize_actor_states.hh"],
    deps = [
        ":visit_topic",
        "//resim/actor:actor_id",
        "//resim/actor/state:observable_state",
        "//resim/actor/state/proto:observable_state_proto_cc",
        "//resim/actor/state/proto:observable_state_to_proto",
        "//resim/assert",
        "//resim/experiences:experience",
        "//resim/experiences:geometry",
        "//resim/simulator:standard_frames",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/transforms:se3",
        "//resim/utils:inout",
        "//resim/utils:match",
        "//resim/utils:mcap_logger",
        "//resim/visualization/foxglove:frame_transform_to_foxglove",
        "//resim/visualization/foxglove:wireframe_to_foxglove",
        "@fmt",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "visualize_actor_states_test",
    srcs = ["visualize_actor_states_test.cc"],
    deps = [
        ":test_helpers",
        ":visit_topic",
        ":visualize_actor_states",
        "//resim/actor/state:observable_state",
        "//resim/actor/state:rigid_body_state",
        "//resim/actor/state/proto:observable_state_proto_cc",
        "//resim/actor/state/proto:observable_state_to_proto",
        "//resim/assert",
        "//resim/curves:two_jet",
        "//resim/curves:two_jet_test_helpers",
        "//resim/experiences:experience",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/experiences/proto:experience_to_proto",
        "//resim/experiences/proto:experiences_test_helpers",
        "//resim/geometry:wireframe",
        "//resim/simulator:standard_frames",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:mcap_logger",
        "//resim/visualization/foxglove:frame_transform_to_foxglove",
        "//resim/visualization/foxglove:wireframe_to_foxglove",
        "@com_google_googletest//:gtest_main",
        "@fmt",
        "@foxglove_schemas",
    ],
)

cc_binary(
    name = "make_visualization_log",
    srcs = ["make_visualization_log.cc"],
    deps = [
        ":extract_experience",
        ":visualize_actor_states",
        ":visualize_world_glb",
        "//resim/assert",
        "//resim/experiences:experience",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/time:timestamp",
        "//resim/utils:inout",
        "//resim/utils:mcap_logger",
        "@cxxopts",
    ],
)

cc_library(
    name = "test_helpers",
    srcs = ["test_helpers.cc"],
    hdrs = ["test_helpers.hh"],
    deps = [
        "//resim/experiences:actor",
        "//resim/experiences:experience",
        "//resim/experiences:geometry",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/geometry:drone_wireframe",
        "//resim/geometry:wireframe",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:mcap_logger",
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "test_helpers_test",
    srcs = ["test_helpers_test.cc"],
    deps = [
        ":test_helpers",
        "//resim/experiences:experience",
        "//resim/experiences/proto:experience_proto_cc",
        "//resim/geometry:wireframe",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:mcap_logger",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "visualize_world_glb",
    srcs = ["visualize_world_glb.cc"],
    hdrs = ["visualize_world_glb.hh"],
    deps = [
        "//resim/simulator:standard_frames",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/utils:inout",
        "//resim/utils:mcap_logger",
        "//resim/visualization/foxglove:pose_to_foxglove",
        "//resim/visualization/foxglove:vector_to_foxglove",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "visualize_world_glb_test",
    srcs = ["visualize_world_glb_test.cc"],
    deps = [
        ":visualize_world_glb",
        "//resim/converter:fuzz_helpers",
        "//resim/simulator:standard_frames",
        "//resim/time/proto:time_to_proto",
        "//resim/utils/testing:mock_logger",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
    ],
)
