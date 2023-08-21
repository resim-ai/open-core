# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "good_test_converter_plugin.so",
    testonly = True,
    srcs = ["good_test_converter_plugin.cc"],
    linkshared = True,
    visibility = ["//resim/msg:__subpackages__"],
    deps = [
        "//resim/assert",
        "//resim/msg:converter_plugin_status",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_binary(
    name = "bad_test_converter_plugin.so",
    testonly = True,
    srcs = ["bad_test_converter_plugin.cc"],
    linkshared = True,
    visibility = ["//resim/msg:__subpackages__"],
    deps = [
        "//resim/msg:converter_plugin_status",
        "@ros2_rclcpp//:rclcpp",
    ],
)
