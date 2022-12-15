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
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "//resim_core/utils:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)
