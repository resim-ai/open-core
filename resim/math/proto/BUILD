# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "matrix_to_proto",
    hdrs = ["matrix_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "//resim/utils:inout",
        "@com_google_protobuf//:protobuf",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "matrix_to_proto_test",
    srcs = ["matrix_to_proto_test.cc"],
    deps = [
        ":matrix_to_proto",
        "//resim/assert",
        "//resim/math/proto/testing:test_matrix_proto_cc",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)
