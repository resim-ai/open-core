load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "d_curve",
    srcs = ["d_curve.cc"],
    hdrs = ["d_curve.hh"],
    deps = [
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
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
    name = "quintic_poly_coeffs",
    srcs = ["quintic_poly_coeffs.cc"],
    hdrs = ["quintic_poly_coeffs.hh"],
)

cc_test(
    name = "quintic_poly_coeffs_test",
    srcs = ["quintic_poly_coeffs_test.cc"],
    deps = [
        ":quintic_poly_coeffs",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "t_curve",
    srcs = ["t_curve.cc"],
    hdrs = ["t_curve.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":t_curve_segment",
        ":two_jet",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:inout",
    ],
)

cc_test(
    name = "t_curve_test",
    srcs = ["t_curve_test.cc"],
    deps = [
        ":t_curve",
        ":two_jet",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "t_curve_segment",
    srcs = ["t_curve_segment.cc"],
    hdrs = ["t_curve_segment.hh"],
    deps = [
        ":quintic_poly_coeffs",
        ":two_jet",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:inout",
    ],
)

cc_test(
    name = "t_curve_segment_test",
    srcs = ["t_curve_segment_test.cc"],
    deps = [
        ":t_curve_segment",
        ":two_jet",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "two_jet",
    srcs = ["two_jet.cc"],
    hdrs = ["two_jet.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":two_jet_concepts",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
    ],
)

cc_test(
    name = "two_jet_test",
    srcs = ["two_jet_test.cc"],
    deps = [
        ":two_jet",
        ":two_jet_concepts",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "two_jet_concepts",
    hdrs = ["two_jet_concepts.hh"],
    deps = [
        "//resim_core/transforms:liegroup_concepts",
    ],
)

cc_test(
    name = "two_jet_concepts_test",
    srcs = ["two_jet_concepts_test.cc"],
    deps = [
        ":two_jet",
        ":two_jet_concepts",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "extrapolate_two_jet",
    srcs = ["extrapolate_two_jet.cc"],
    hdrs = ["extrapolate_two_jet.hh"],
    deps = [
        ":two_jet",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
    ],
)

cc_test(
    name = "extrapolate_two_jet_test",
    srcs = ["extrapolate_two_jet_test.cc"],
    deps = [
        ":extrapolate_two_jet",
        ":two_jet",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)
