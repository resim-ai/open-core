load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.hh"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "uuid_test",
    srcs = ["uuid_test.cc"],
    deps = [
        ":uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inout",
    hdrs = ["inout.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "inout_test",
    srcs = ["inout_test.cc"],
    deps = [
        ":inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "random_vector",
    testonly = True,
    hdrs = ["random_vector.hh"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_test(
    name = "random_vector_test",
    srcs = ["random_vector_test.cc"],
    deps = [
        ":random_vector",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "double_buffer",
    hdrs = ["double_buffer.hh"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_test(
    name = "double_buffer_test",
    srcs = ["double_buffer_test.cc"],
    deps = [
        ":double_buffer",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "type",
    hdrs = ["type.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "type_test",
    srcs = ["type_test.cc"],
    deps = [
        ":type",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "integer_power",
    srcs = ["integer_power.cc"],
    hdrs = ["integer_power.hh"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_test(
    name = "integer_power_test",
    srcs = ["integer_power_test.cc"],
    deps = [
        ":integer_power",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "mcap_logger",
    srcs = ["mcap_logger.cc"],
    hdrs = ["mcap_logger.hh"],
    deps = [
        "//resim_core/third_party/mcap:mcap_impl",
        "//resim_core/time:timestamp",
        "//resim_core/utils/proto:dependency_file_descriptor_set",
    ],
)

cc_test(
    name = "mcap_logger_test",
    srcs = ["mcap_logger_test.cc"],
    deps = [
        ":mcap_logger",
        "//resim_core/utils/proto/testing:message_a_proto_cc",
        "//resim_core/utils/proto/testing:test_proto_cc",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "move_copy_tracker",
    testonly = True,
    srcs = ["move_copy_tracker.cc"],
    hdrs = ["move_copy_tracker.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "move_copy_tracker_test",
    srcs = ["move_copy_tracker_test.cc"],
    deps = [
        ":move_copy_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "match",
    hdrs = ["match.hh"],
    deps = [
    ],
)

cc_test(
    name = "match_test",
    srcs = ["match_test.cc"],
    deps = [
        ":match",
        ":move_copy_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "nullable_reference",
    hdrs = ["nullable_reference.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "nullable_reference_test",
    srcs = ["nullable_reference_test.cc"],
    deps = [
        ":nullable_reference",
        "@com_google_googletest//:gtest_main",
    ],
)
