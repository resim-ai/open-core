load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "auth_messages_proto",
    srcs = ["auth_messages.proto"],
    deps = [
        "//resim_core/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "auth_messages_proto_cc",
    visibility = ["//resim_core/auth:__subpackages__"],
    deps = [":auth_messages_proto"],
)
