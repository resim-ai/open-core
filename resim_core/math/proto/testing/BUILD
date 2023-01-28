load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "test_matrix_proto",
    testonly = True,
    srcs = ["test_matrix.proto"],
)

cc_proto_library(
    name = "test_matrix_proto_cc",
    testonly = True,
    visibility = ["//resim_core/math/proto:__pkg__"],
    deps = [":test_matrix_proto"],
)
