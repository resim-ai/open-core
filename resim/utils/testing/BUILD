# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "mock_logger",
    testonly = True,
    srcs = ["mock_logger.cc"],
    hdrs = ["mock_logger.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "//resim/time:timestamp",
        "//resim/utils:mcap_logger",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "mock_logger_test",
    srcs = ["mock_logger_test.cc"],
    deps = [
        ":mock_logger",
        "//resim/time:timestamp",
        "//resim/transforms/proto:se3_proto_cc",
        "//resim/transforms/proto:so3_proto_cc",
        "//resim/utils:uuid",
        "//resim/utils/proto:uuid_proto_cc",
        "//resim/utils/proto:uuid_to_proto",
        "@com_google_googletest//:gtest_main",
    ],
)
