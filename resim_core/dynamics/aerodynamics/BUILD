load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "rigid_body_aerodynamics",
    srcs = ["rigid_body_aerodynamics.cc"],
    hdrs = ["rigid_body_aerodynamics.hh"],
    deps = [
        "//resim_core/actor/state:rigid_body_state",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_vector",
        "//resim_core/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "rigid_body_aerodynamics_test",
    srcs = ["rigid_body_aerodynamics_test.cc"],
    deps = [
        ":rigid_body_aerodynamics",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
