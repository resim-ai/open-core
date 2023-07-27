load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

exports_files(
    [
        "quintic_poly_coeffs.hh",
        "t_curve.hh",
        "t_curve_segment.hh",
        "two_jet.hh",
        "two_jet_concepts.hh",
        "d_curve.hh",
        "quintic_poly_coeffs.cc",
        "t_curve.cc",
        "t_curve_segment.cc",
        "two_jet.cc",
        "two_jet_concepts.cc",
        "d_curve.cc",
    ],
)

cc_library(
    name = "d_curve",
    srcs = ["d_curve.cc"],
    hdrs = ["d_curve.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "//resim/transforms:se3",
    ],
)

cc_library(
    name = "d_curve_test_helpers",
    testonly = 1,
    srcs = ["d_curve_test_helpers.cc"],
    hdrs = ["d_curve_test_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":d_curve",
        "//resim/assert",
        "//resim/transforms:frame",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "d_curve_test",
    srcs = ["d_curve_test.cc"],
    deps = [
        ":d_curve",
        ":d_curve_test_helpers",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "quintic_poly_coeffs",
    srcs = ["quintic_poly_coeffs.cc"],
    hdrs = ["quintic_poly_coeffs.hh"],
    visibility = ["//resim/curves:__subpackages__"],
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
        "//resim/assert",
        "//resim/transforms:frame",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "t_curve_test",
    srcs = ["t_curve_test.cc"],
    deps = [
        ":t_curve",
        ":t_curve_test_helpers",
        ":two_jet",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
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
        "//resim/assert",
        "//resim/transforms:frame",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "t_curve_segment_test",
    srcs = ["t_curve_segment_test.cc"],
    deps = [
        ":t_curve_segment",
        ":two_jet",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
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
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
    ],
)

cc_test(
    name = "two_jet_test",
    srcs = ["two_jet_test.cc"],
    deps = [
        ":two_jet",
        ":two_jet_concepts",
        ":two_jet_test_helpers",
        "//resim/testing:random_matrix",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "two_jet_concepts",
    hdrs = ["two_jet_concepts.hh"],
    deps = [
        "//resim/transforms:liegroup_concepts",
    ],
)

cc_test(
    name = "two_jet_concepts_test",
    srcs = ["two_jet_concepts_test.cc"],
    deps = [
        ":two_jet",
        ":two_jet_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "extrapolate_two_jet",
    srcs = ["extrapolate_two_jet.cc"],
    hdrs = ["extrapolate_two_jet.hh"],
    deps = [
        ":two_jet",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
    ],
)

cc_test(
    name = "extrapolate_two_jet_test",
    srcs = ["extrapolate_two_jet_test.cc"],
    deps = [
        ":extrapolate_two_jet",
        ":two_jet",
        ":two_jet_test_helpers",
        "//resim/testing:random_matrix",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "two_jet_test_helpers",
    testonly = 1,
    srcs = ["two_jet_test_helpers.cc"],
    hdrs = ["two_jet_test_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":two_jet",
        ":two_jet_concepts",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
    ],
)

cc_test(
    name = "two_jet_test_helpers_test",
    srcs = ["two_jet_test_helpers_test.cc"],
    deps = [
        ":two_jet_test_helpers",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "t_curve_test_helpers",
    testonly = 1,
    srcs = ["t_curve_test_helpers.cc"],
    hdrs = ["t_curve_test_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":t_curve",
        ":two_jet",
        ":two_jet_test_helpers",
        "//resim/transforms:frame",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "t_curve_test_helpers_test",
    srcs = ["t_curve_test_helpers_test.cc"],
    deps = [
        ":t_curve",
        ":t_curve_test_helpers",
        ":two_jet",
        ":two_jet_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "test_helpers",
    testonly = True,
    srcs = ["test_helpers.cc"],
    hdrs = ["test_helpers.hh"],
    visibility = ["//resim:__subpackages__"],
    deps = [
        ":t_curve",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "test_helpers_test",
    srcs = ["test_helpers_test.cc"],
    deps = [
        ":test_helpers",
        "//resim/curves:t_curve",
        "//resim/transforms:frame",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
