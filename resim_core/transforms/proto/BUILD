load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "frame_3_proto",
    srcs = ["frame_3.proto"],
    deps = [
        "//resim_core/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "frame_3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":frame_3_proto"],
)

cc_library(
    name = "frame_3_to_proto",
    srcs = ["frame_3_to_proto.cc"],
    hdrs = ["frame_3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_proto_cc",
        "//resim_core/transforms:frame",
        "//resim_core/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "frame_3_to_proto_test",
    srcs = ["frame_3_to_proto_test.cc"],
    deps = [
        ":frame_3_proto_cc",
        ":frame_3_to_proto",
        "//resim_core/transforms:frame",
        "@com_google_googletest//:gtest_main",
    ],
)
