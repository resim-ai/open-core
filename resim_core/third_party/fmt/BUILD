load("@rules_cc//cc:defs.bzl", "cc_test")

exports_files(
    glob(["*.BUILD"]),
    visibility = ["//visibility:public"],
)

cc_test(
    name = "fmt_test",
    srcs = ["fmt_test.cc"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "@fmt",
    ],
)
