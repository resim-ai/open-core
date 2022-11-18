load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "test_proto",
    srcs = ["test.proto"],
)

cc_proto_library(
    name = "test_proto_cc",
    testonly = 1,
    visibility = ["//visibility:public"],
    deps = [":test_proto"],
)
