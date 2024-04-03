# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "fuzz_helpers",
    testonly = True,
    srcs = ["fuzz_helpers.cc"],
    hdrs = [
        "fuzz_helpers.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":parser",
        ":tags",
        "//resim/math:is_approx",
        "//resim/testing:random_matrix",
        "//resim/time:timestamp",
        "//resim/utils:inout",
        "//resim/utils:match",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "fuzz_helpers_test_template",
    testonly = True,
    hdrs = ["fuzz_helpers_test_template.hh"],
    deps = [
        ":fuzz_helpers",
    ],
)

cc_test(
    name = "fuzz_helpers_test",
    srcs = ["fuzz_helpers_test.cc"],
    deps = [
        ":fuzz_helpers",
        ":fuzz_helpers_test_template",
        "//resim/converter/testing:test_message_proto_cc",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "tags",
    hdrs = ["tags.hh"],
    deps = [
    ],
)

cc_library(
    name = "parser",
    hdrs = ["parser.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":tags",
        "//resim/utils:setter_reference",
    ],
)

cc_test(
    name = "parser_test",
    srcs = ["parser_test.cc"],
    deps = [
        ":parser",
        "@com_google_googletest//:gtest_main",
    ],
)
