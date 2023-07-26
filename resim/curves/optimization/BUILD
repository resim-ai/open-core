load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "two_jet_tangent_space",
    srcs = ["two_jet_tangent_space.cc"],
    hdrs = ["two_jet_tangent_space.hh"],
    deps = [
        "//resim/curves:two_jet",
        "//resim/math:vector_partition",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
    ],
)

cc_test(
    name = "two_jet_tangent_space_test",
    srcs = ["two_jet_tangent_space_test.cc"],
    deps = [
        ":two_jet_tangent_space",
        "//resim/curves:two_jet",
        "//resim/curves:two_jet_test_helpers",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)
