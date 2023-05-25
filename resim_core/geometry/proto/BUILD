load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "wireframe_proto",
    srcs = ["wireframe.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "wireframe_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":wireframe_proto"],
)

cc_library(
    name = "wireframe_to_proto",
    srcs = ["wireframe_to_proto.cc"],
    hdrs = ["wireframe_to_proto.hh"],
    deps = [
        ":wireframe_proto_cc",
        "//resim_core/assert",
        "//resim_core/geometry:wireframe",
        "//resim_core/math/proto:matrix_to_proto",
    ],
)

cc_test(
    name = "wireframe_to_proto_test",
    srcs = ["wireframe_to_proto_test.cc"],
    deps = [
        ":wireframe_proto_cc",
        ":wireframe_to_proto",
        "//resim_core/assert",
        "//resim_core/geometry:wireframe",
        "//resim_core/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)
