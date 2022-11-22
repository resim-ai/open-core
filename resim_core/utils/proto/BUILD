load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "dependency_file_descriptor_set",
    srcs = ["dependency_file_descriptor_set.cc"],
    hdrs = ["dependency_file_descriptor_set.hh"],
    visibility = ["//visibility:public"],
    deps = [
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
