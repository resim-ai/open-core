# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

exports_files(
    glob(["*.BUILD"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "mcap_impl",
    srcs = ["mcap_impl.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "@mcap",
    ],
)

cc_test(
    name = "mcap_test",
    srcs = ["mcap_test.cc"],
    deps = [
        ":mcap_impl",
        "//resim/testing:test_directory",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
        "@mcap",
    ],
)
