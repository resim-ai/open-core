load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "channel",
    hdrs = ["channel.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
    ],
)

cc_test(
    name = "channel_test",
    srcs = ["channel_test.cc"],
    deps = [
        ":channel",
        "@com_google_googletest//:gtest_main",
    ],
)
