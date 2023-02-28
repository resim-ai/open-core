load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "matrix_to_proto",
    hdrs = ["matrix_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/assert",
        "//resim_core/utils:inout",
        "@com_google_protobuf//:protobuf",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "matrix_to_proto_test",
    srcs = ["matrix_to_proto_test.cc"],
    deps = [
        ":matrix_to_proto",
        "//resim_core/assert",
        "//resim_core/math/proto/testing:test_matrix_proto_cc",
        "//resim_core/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)
