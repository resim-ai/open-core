load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "polynomial",
    srcs = ["polynomial.cc"],
    hdrs = ["polynomial.hh"],
    deps = [],
)

cc_test(
    name = "polynomial_test",
    srcs = ["polynomial_test.cc"],
    deps = [
        ":polynomial",
        "@com_google_googletest//:gtest_main",
    ],
)
