load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "liblz4",
    srcs = [
        "lib/lz4.c",
        "lib/lz4frame.c",
        "lib/lz4frame_static.h",
        "lib/lz4hc.c",
        "lib/xxhash.c",
        "lib/xxhash.h",
    ],
    hdrs = [
        "lib/lz4.c",
        "lib/lz4.h",
        "lib/lz4frame.h",
        "lib/lz4hc.h",
    ],
    strip_include_prefix = "lib/",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "lz4_lz4c_include",
    hdrs = [
        "lib/lz4.c",
    ],
    strip_include_prefix = "lib/",
)

cc_library(
    name = "lz4_hc",
    srcs = [
        "lib/lz4hc.c",
    ],
    hdrs = [
        "lib/lz4hc.h",
    ],
    strip_include_prefix = "lib/",
    visibility = ["//visibility:public"],
    deps = [
        ":lz4",
        ":lz4_lz4c_include",
    ],
)

cc_library(
    name = "lz4_frame",
    srcs = [
        "lib/lz4frame.c",
        "lib/lz4frame_static.h",
    ],
    hdrs = [
        "lib/lz4frame.h",
    ],
    strip_include_prefix = "lib/",
    visibility = ["//visibility:public"],
    deps = [
        ":lz4",
        ":lz4_hc",
    ],
)
