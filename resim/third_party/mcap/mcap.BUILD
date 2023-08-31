load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "mcap",
    hdrs = glob(["include/mcap/**"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@lz4//:liblz4",
        "@zstd//:libzstd",
    ],
)
