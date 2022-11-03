load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

exports_files(
    glob(["*.BUILD"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "mcap_impl",
    srcs = ["mcap_impl.cc"],
    deps = [
        "@mcap",
    ],
)

cc_test(
    name = "mcap_test",
    srcs = ["mcap_test.cc"],
    deps = [
        ":mcap_impl",
        "//resim_core/utils:uuid",
        "@com_google_googletest//:gtest_main",
        "@mcap",
    ],
)
