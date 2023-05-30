load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mock_logger",
    testonly = True,
    srcs = ["mock_logger.cc"],
    hdrs = ["mock_logger.hh"],
    deps = [
        "//resim_core/assert",
        "//resim_core/time:timestamp",
        "//resim_core/utils:mcap_logger",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "mock_logger_test",
    srcs = ["mock_logger_test.cc"],
    deps = [
        ":mock_logger",
        "//resim_core/time:timestamp",
        "//resim_core/transforms/proto:se3_proto_cc",
        "//resim_core/transforms/proto:so3_proto_cc",
        "//resim_core/utils:uuid",
        "//resim_core/utils/proto:uuid_proto_cc",
        "//resim_core/utils/proto:uuid_to_proto",
        "@com_google_googletest//:gtest_main",
    ],
)
