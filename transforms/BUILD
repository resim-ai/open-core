load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "so3",
    srcs = ["so3.cc"],
    hdrs = ["so3.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "so3_test",
    srcs = ["so3_test.cc"],
    deps = [
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
