# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

exports_files(
    [
        "inout.hh",
        "uuid.hh",
        "inout.cc",
        "uuid.cc",
    ],
)

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "@libuuid",
    ],
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
    deps = [
        "//resim/assert",
    ],
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
    visibility = ["//visibility:public"],
    deps = [
        ":inout",
        "//resim/assert",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/time:timestamp",
        "//resim/utils/proto:dependency_file_descriptor_set",
    ],
)

cc_test(
    name = "mcap_logger_test",
    srcs = ["mcap_logger_test.cc"],
    deps = [
        ":inout",
        ":mcap_logger",
        "//resim/assert",
        "//resim/testing:test_directory",
        "//resim/utils/proto/testing:message_a_proto_cc",
        "//resim/utils/proto/testing:test_proto_cc",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "match",
    hdrs = ["match.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "match_test",
    srcs = ["match_test.cc"],
    deps = [
        ":match",
        "//resim/testing:move_copy_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "nullable_reference",
    hdrs = ["nullable_reference.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
    ],
)

cc_test(
    name = "nullable_reference_test",
    srcs = ["nullable_reference_test.cc"],
    deps = [
        ":nullable_reference",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "status",
    srcs = ["status.cc"],
    hdrs = ["status.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":match",
        "//resim/assert",
        "@fmt",
    ],
)

cc_test(
    name = "status_test",
    srcs = ["status_test.cc"],
    deps = [
        ":status",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
        "@fmt",
    ],
)

cc_library(
    name = "status_value",
    hdrs = ["status_value.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":match",
        ":status",
        "//resim/assert",
    ],
)

cc_test(
    name = "status_value_test",
    srcs = ["status_value_test.cc"],
    deps = [
        ":status_value",
        "//resim/assert",
        "//resim/testing:move_copy_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "http_response",
    hdrs = ["http_response.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_library(
    name = "tuple_utils",
    hdrs = ["tuple_utils.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":type",
    ],
)

cc_test(
    name = "tuple_utils_test",
    srcs = ["tuple_utils_test.cc"],
    deps = [
        ":tuple_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "mcap_test_helpers",
    testonly = 1,
    srcs = ["mcap_test_helpers.cc"],
    hdrs = ["mcap_test_helpers.hh"],
    deps = [
        "//resim/third_party/mcap:mcap_impl",
    ],
)

cc_test(
    name = "mcap_test_helpers_test",
    srcs = ["mcap_test_helpers_test.cc"],
    deps = [
        ":mcap_logger",
        ":mcap_test_helpers",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils/proto/testing:message_a_proto_cc",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "snippet_mcap",
    srcs = ["snippet_mcap.cc"],
    hdrs = ["snippet_mcap.hh"],
    deps = [
        "//resim/assert",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/time:timestamp",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "snippet_mcap_test",
    srcs = ["snippet_mcap_test.cc"],
    deps = [
        ":inout",
        ":mcap_logger",
        ":mcap_test_helpers",
        ":snippet_mcap",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils/proto/testing:message_a_proto_cc",
        "//resim/utils/proto/testing:message_b_proto_cc",
        "//resim/utils/proto/testing:message_c_proto_cc",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "trim_mcap",
    srcs = ["trim_mcap.cc"],
    deps = [
        ":snippet_mcap",
        "@cxxopts",
        "@fmt",
    ],
)

cc_library(
    name = "setter_reference",
    hdrs = ["setter_reference.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "setter_reference_test",
    srcs = ["setter_reference_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":setter_reference",
        "@com_google_googletest//:gtest_main",
    ],
)
