load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
)

cmake(
    name = "libzstd",
    cache_entries = {
        "ZSTD_BUILD_PROGRAMS": "OFF",
        "ZSTD_BUILD_SHARED": "OFF",
        "ZSTD_BUILD_STATIC": "ON",
    },
    lib_source = ":all_srcs",
    out_static_libs = ["libzstd.a"],
    visibility = ["//visibility:public"],
    working_directory = "build/cmake",
)

# The @ros2_rosbag2 repository requires this target be present in @libzstd
alias(
    name = "zstd",
    actual = ":libzstd",
    visibility = ["//visibility:public"],
)
