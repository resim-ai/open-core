load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "d_curve",
    srcs = ["d_curve.cc"],
    hdrs = ["d_curve.hh"],
    deps = [
        "//transforms:framed_group",
        "//transforms:se3",
    ],
)

cc_test(
    name = "d_curve_test",
    srcs = ["d_curve_test.cc"],
    deps = [
        ":d_curve",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "two_jet",
    srcs = ["two_jet.cc"],
    hdrs = ["two_jet.hh"],
    deps = [
        "//transforms:framed_group",
        "//transforms:liegroup_concepts",
        "//transforms:se3",
    ],
)

cc_test(
    name = "two_jet_test",
    srcs = ["two_jet_test.cc"],
    deps = [
        ":two_jet",
        "//transforms:framed_group",
        "//transforms:se3",
        "//transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)
