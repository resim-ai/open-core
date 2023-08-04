load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "view_server_test_helper",
    testonly = 1,
    srcs = ["view_server_test_helper.cc"],
    hdrs = ["view_server_test_helper.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:d_curve_test_helpers",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/testing:random_matrix",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
    ],
)

cc_test(
    name = "view_server_test_helper_test",
    testonly = 1,
    srcs = ["view_server_test_helper_test.cc"],
    deps = [
        ":view_server_test_helper",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/curves:d_curve",
        "//resim/curves:d_curve_test_helpers",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/transforms:frame",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)
