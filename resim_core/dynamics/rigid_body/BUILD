load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "inertia",
    srcs = ["inertia.cc"],
    hdrs = ["inertia.hh"],
    deps = [
        "//resim_core/transforms:framed_group",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "inertia_test",
    srcs = ["inertia_test.cc"],
    deps = [
        ":inertia",
        "//resim_core/testing:random_matrix",
        "//resim_core/transforms:framed_group",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
