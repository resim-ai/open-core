load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "rigid_body_aerodynamics",
    srcs = ["rigid_body_aerodynamics.cc"],
    hdrs = ["rigid_body_aerodynamics.hh"],
    deps = [
        "//resim/actor/state:rigid_body_state",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/utils:nullable_reference",
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

cc_library(
    name = "airfoil",
    srcs = ["airfoil.cc"],
    hdrs = ["airfoil.hh"],
    deps = [
        ":drag_coefficients",
        ":rigid_body_aerodynamics",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "airfoil_test",
    srcs = ["airfoil_test.cc"],
    deps = [
        ":airfoil",
        ":rigid_body_aerodynamics",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "drag_coefficients",
    hdrs = ["drag_coefficients.hh"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "drag_coefficients_test",
    srcs = ["drag_coefficients_test.cc"],
    deps = [
        ":drag_coefficients",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
