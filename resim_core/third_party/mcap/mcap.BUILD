load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "mcap",
    hdrs = glob(["cpp/mcap/include/mcap/**"]),
    includes = ["cpp/mcap/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@lz4//:liblz4",
        "@zstd//:libzstd",
    ],
)
