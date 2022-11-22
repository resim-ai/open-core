load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "test_proto",
    testonly = 1,
    srcs = ["test.proto"],
)

cc_proto_library(
    name = "test_proto_cc",
    testonly = 1,
    visibility = ["//visibility:public"],
    deps = [":test_proto"],
)

proto_library(
    name = "message_a_proto",
    testonly = 1,
    srcs = ["message_a.proto"],
)

cc_proto_library(
    name = "message_a_proto_cc",
    testonly = 1,
    visibility = ["//visibility:public"],
    deps = [":message_a_proto"],
)

proto_library(
    name = "message_b_proto",
    testonly = 1,
    srcs = ["message_b.proto"],
    deps = [":message_a_proto"],
)

cc_proto_library(
    name = "message_b_proto_cc",
    testonly = 1,
    visibility = ["//visibility:public"],
    deps = [":message_b_proto"],
)

proto_library(
    name = "message_c_proto",
    testonly = 1,
    srcs = ["message_c.proto"],
    deps = [":message_a_proto"],
)

cc_proto_library(
    name = "message_c_proto_cc",
    testonly = 1,
    visibility = ["//visibility:public"],
    deps = [":message_c_proto"],
)

proto_library(
    name = "message_d_proto",
    testonly = 1,
    srcs = ["message_d.proto"],
    deps = [
        ":message_b_proto",
        ":message_c_proto",
    ],
)

cc_proto_library(
    name = "message_d_proto_cc",
    testonly = 1,
    visibility = ["//visibility:public"],
    deps = [":message_d_proto"],
)
