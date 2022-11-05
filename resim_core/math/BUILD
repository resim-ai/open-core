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

cc_library(
    name = "multivariate_gaussian",
    hdrs = ["multivariate_gaussian.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "multivariate_gaussian_test",
    srcs = ["multivariate_gaussian_test.cc"],
    deps = [
        ":multivariate_gaussian",
        ":sample_statistics",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sample_statistics",
    hdrs = ["sample_statistics.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "sample_statistics_test",
    srcs = ["sample_statistics_test.cc"],
    deps = [
        ":sample_statistics",
        "@com_google_googletest//:gtest_main",
    ],
)
