load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "so3",
    hdrs = ["so3.hh"],
    srcs = ["so3.cc"],
    deps = [
        "@libeigen//:eigen"
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "so3_test",
    srcs = ["so3_test.cc"],
    deps = [
        ":so3",
        "@libeigen//:eigen",
        "@com_google_googletest//:gtest_main"
    ],
)
