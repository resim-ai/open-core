load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.hh"],
)

cc_test(
    name = "uuid_test",
    srcs = ["uuid_test.cc"],
    deps = [
        ":uuid",
        "@com_google_googletest//:gtest_main",
    ],
)
