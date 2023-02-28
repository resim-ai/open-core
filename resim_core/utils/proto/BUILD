load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

cc_library(
    name = "dependency_file_descriptor_set",
    srcs = ["dependency_file_descriptor_set.cc"],
    hdrs = ["dependency_file_descriptor_set.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/assert",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "dependency_file_descriptor_set_test",
    srcs = ["dependency_file_descriptor_set_test.cc"],
    deps = [
        ":dependency_file_descriptor_set",
        "//resim_core/utils/proto/testing:message_a_proto_cc",
        "//resim_core/utils/proto/testing:message_b_proto_cc",
        "//resim_core/utils/proto/testing:message_c_proto_cc",
        "//resim_core/utils/proto/testing:message_d_proto_cc",
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

cc_library(
    name = "uuid_to_proto",
    srcs = ["uuid_to_proto.cc"],
    hdrs = ["uuid_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":uuid_proto_cc",
        "//resim_core/assert",
        "//resim_core/utils:uuid",
    ],
)

cc_test(
    name = "uuid_to_proto_test",
    srcs = ["uuid_to_proto_test.cc"],
    deps = [
        ":uuid_proto_cc",
        ":uuid_to_proto",
        "//resim_core/assert",
        "//resim_core/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)
