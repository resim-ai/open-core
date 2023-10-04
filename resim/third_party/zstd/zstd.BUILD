load("@rules_foreign_cc//foreign_cc:defs.bzl", "make")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
)

make(
    name = "libzstd",
    lib_source = ":all_srcs",
    visibility = ["//visibility:public"],
)

# The @ros2_rosbag2 repository requires this target be present in @libzstd
alias(
    name = "zstd",
    actual = ":libzstd",
    visibility = ["//visibility:public"],
)
