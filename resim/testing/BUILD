# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "test_directory",
    testonly = True,
    srcs = ["test_directory.cc"],
    hdrs = ["test_directory.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "test_directory_test",
    srcs = ["test_directory_test.cc"],
    deps = [
        ":test_directory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "random_matrix",
    hdrs = ["random_matrix.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "random_matrix_test",
    srcs = ["random_matrix_test.cc"],
    deps = [
        ":random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
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
    name = "mock_server",
    testonly = 1,
    srcs = ["mock_server.cc"],
    hdrs = ["mock_server.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "//resim/utils:http_response",
        "//resim/utils:inout",
        "@com_github_google_glog//:glog",
        "@httplib",
    ],
)

cc_library(
    name = "fuzz_helpers",
    testonly = True,
    srcs = ["fuzz_helpers.cc"],
    hdrs = ["fuzz_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/math:is_approx",
        "//resim/utils:inout",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "fuzz_helpers_test",
    srcs = ["fuzz_helpers_test.cc"],
    deps = [
        ":fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)
