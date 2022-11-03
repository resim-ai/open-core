load("@rules_foreign_cc//foreign_cc:defs.bzl", "make")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
)

make(
    name = "liblz4",
    lib_source = ":all_srcs",
    visibility = ["//visibility:public"],
)
