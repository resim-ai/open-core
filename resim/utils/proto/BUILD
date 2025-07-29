# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")

load("@rules_go//proto:def.bzl", "go_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

cc_library(
    name = "dependency_file_descriptor_set",
    srcs = ["dependency_file_descriptor_set.cc"],
    hdrs = ["dependency_file_descriptor_set.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "dependency_file_descriptor_set_test",
    srcs = ["dependency_file_descriptor_set_test.cc"],
    deps = [
        ":dependency_file_descriptor_set",
        "//resim/metrics/proto:simple_metric_proto_cc",
        "//resim/utils/proto/testing:message_a_proto_cc",
        "//resim/utils/proto/testing:message_b_proto_cc",
        "//resim/utils/proto/testing:message_c_proto_cc",
        "//resim/utils/proto/testing:message_d_proto_cc",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

proto_library(
    name = "uuid_proto",
    srcs = ["uuid.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "uuid_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":uuid_proto"],
)

py_proto_library(
    name = "uuid_proto_py",
    visibility = ["//visibility:public"],
    deps = [":uuid_proto"],
)

go_proto_library(
    name = "uuid_proto_go",
    importpath = "github.com/resim-ai/open-core/uuid_proto",
    protos = [":uuid_proto"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "uuid_to_proto",
    srcs = ["uuid_to_proto.cc"],
    hdrs = ["uuid_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":uuid_proto_cc",
        "//resim/assert",
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "uuid_to_proto_test",
    srcs = ["uuid_to_proto_test.cc"],
    deps = [
        ":uuid_proto_cc",
        ":uuid_to_proto",
        "//resim/assert",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)
