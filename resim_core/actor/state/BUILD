load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "rigid_body_state",
    srcs = ["rigid_body_state.cc"],
    hdrs = ["rigid_body_state.hh"],
    deps = [
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "rigid_body_state_test",
    srcs = ["rigid_body_state_test.cc"],
    deps = [
        ":rigid_body_state",
        "//resim_core/curves:two_jet",
        "//resim_core/testing:random_matrix",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "trajectory",
    srcs = ["trajectory.cc"],
    hdrs = ["trajectory.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":rigid_body_state",
        "//resim_core/assert",
        "//resim_core/curves:t_curve",
        "//resim_core/curves:two_jet",
        "//resim_core/time:timestamp",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = ["trajectory_test.cc"],
    deps = [
        ":rigid_body_state",
        ":trajectory",
        "//resim_core/assert",
        "//resim_core/curves:t_curve",
        "//resim_core/curves:two_jet",
        "//resim_core/testing:random_matrix",
        "//resim_core/time:timestamp",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)
