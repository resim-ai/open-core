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
    visibility = ["//resim/ros2:__subpackages__"],
    deps = [
        "//resim/assert",
        "//resim/ros2:converter_plugin_types",
        "@ros2_rclcpp//:rclcpp",
    ],
)
