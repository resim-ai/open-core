# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "ros2_ament_setup")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary", "ros2_cpp_test")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files")
load("@rules_python//python:defs.bzl", "py_test")
load("//bazel:python.bzl", "pybind_extension")
load(":ros2_py_package.bzl", "ros2_py_package")

cc_library(
    name = "primitives_from_ros2",
    srcs = ["primitives_from_ros2.cc"],
    hdrs = ["primitives_from_ros2.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "//resim/msg:byte_swap_helpers",
        "//resim/msg:primitives_proto_cc",
        "@ros2_common_interfaces//:cpp_std_msgs",
    ],
)

cc_test(
    name = "primitives_from_ros2_test",
    srcs = ["primitives_from_ros2_test.cc"],
    deps = [
        ":primitives_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/msg:fuzz_helpers",
        "//resim/msg:primitives_proto_cc",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "time_from_ros2",
    srcs = ["time_from_ros2.cc"],
    hdrs = ["time_from_ros2.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_protobuf//:protobuf",
        "@ros2_rcl_interfaces//:cpp_builtin_interfaces",
    ],
)

cc_test(
    name = "time_from_ros2_test",
    srcs = ["time_from_ros2_test.cc"],
    deps = [
        ":time_from_ros2",
        "@com_google_googletest//:gtest_main",
        "@ros2_rcl_interfaces//:cpp_builtin_interfaces",
    ],
)

cc_library(
    name = "header_from_ros2",
    srcs = ["header_from_ros2.cc"],
    hdrs = ["header_from_ros2.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":time_from_ros2",
        "//resim/msg:header_proto_cc",
        "@ros2_common_interfaces//:cpp_std_msgs",
    ],
)

cc_test(
    name = "header_from_ros2_test",
    srcs = ["header_from_ros2_test.cc"],
    deps = [
        ":header_from_ros2",
        "@com_google_googletest//:gtest_main",
        "@ros2_common_interfaces//:cpp_std_msgs",
    ],
)

cc_library(
    name = "transform_from_ros2",
    srcs = ["transform_from_ros2.cc"],
    hdrs = ["transform_from_ros2.hh"],
    visibility = ["//resim/ros2:__subpackages__"],
    deps = [
        ":header_from_ros2",
        "//resim/msg:transform_proto_cc",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/transforms/proto:se3_proto_cc",
        "//resim/transforms/proto:se3_to_proto",
        "@libeigen//:eigen",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_geometry2//:cpp_tf2_msgs",
    ],
)

cc_test(
    name = "transform_from_ros2_test",
    srcs = ["transform_from_ros2_test.cc"],
    deps = [
        ":transform_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/msg:fuzz_helpers",
        "//resim/msg:transform_proto_cc",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "pose_from_ros2",
    srcs = ["pose_from_ros2.cc"],
    hdrs = ["pose_from_ros2.hh"],
    deps = [
        "//resim/assert",
        "//resim/msg:pose_proto_cc",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/transforms/proto:se3_proto_cc",
        "//resim/transforms/proto:se3_to_proto",
        "@libeigen//:eigen",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
    ],
)

cc_test(
    name = "pose_from_ros2_test",
    srcs = ["pose_from_ros2_test.cc"],
    deps = [
        ":pose_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/msg:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "point_from_ros2",
    srcs = ["point_from_ros2.cc"],
    hdrs = ["point_from_ros2.hh"],
    deps = [
        "//resim/transforms:framed_vector",
        "//resim/transforms/proto:framed_vector_3_proto_cc",
        "//resim/transforms/proto:framed_vector_3_to_proto",
        "@libeigen//:eigen",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
    ],
)

cc_test(
    name = "point_from_ros2_test",
    srcs = ["point_from_ros2_test.cc"],
    deps = [
        ":point_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/transforms:frame",
        "//resim/transforms/proto:fuzz_helpers",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "odometry_from_ros2",
    srcs = ["odometry_from_ros2.cc"],
    hdrs = ["odometry_from_ros2.hh"],
    visibility = ["//resim/ros2:__subpackages__"],
    deps = [
        ":header_from_ros2",
        ":pose_from_ros2",
        "//resim/msg:odometry_proto_cc",
        "@ros2_common_interfaces//:cpp_nav_msgs",
    ],
)

cc_test(
    name = "odometry_from_ros2_test",
    srcs = ["odometry_from_ros2_test.cc"],
    deps = [
        ":odometry_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/msg:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "oriented_box_from_ros2",
    srcs = ["oriented_box_from_ros2.cc"],
    hdrs = ["oriented_box_from_ros2.hh"],
    deps = [
        ":pose_from_ros2",
        "//resim/assert",
        "//resim/geometry/proto:oriented_box_proto_cc",
        "@vision_msgs//:cpp_vision_msgs",
    ],
)

cc_test(
    name = "oriented_box_from_ros2_test",
    srcs = ["oriented_box_from_ros2_test.cc"],
    deps = [
        ":oriented_box_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/geometry/proto:fuzz_helpers",
        "//resim/transforms:frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "detection_from_ros2",
    srcs = ["detection_from_ros2.cc"],
    hdrs = ["detection_from_ros2.hh"],
    deps = [
        ":header_from_ros2",
        ":oriented_box_from_ros2",
        "//resim/msg:detection_proto_cc",
        "@vision_msgs//:cpp_vision_msgs",
    ],
)

cc_test(
    name = "detection_from_ros2_test",
    srcs = ["detection_from_ros2_test.cc"],
    deps = [
        ":detection_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/msg:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "navsat_from_ros2",
    srcs = ["navsat_from_ros2.cc"],
    hdrs = ["navsat_from_ros2.hh"],
    deps = [
        ":header_from_ros2",
        "//resim/assert",
        "//resim/msg:navsat_proto_cc",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
    ],
)

cc_test(
    name = "navsat_from_ros2_test",
    srcs = ["navsat_from_ros2_test.cc"],
    deps = [
        ":navsat_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/msg:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "converter_plugin_types",
    hdrs = ["converter_plugin_types.h"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_library(
    name = "converter_plugin",
    srcs = ["converter_plugin.cc"],
    hdrs = ["converter_plugin.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":converter_plugin_types",
        "//resim/assert",
        "@com_google_googletest//:gtest_prod",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_test(
    name = "converter_plugin_test",
    srcs = ["converter_plugin_test.cc"],
    data = [
        "//resim/ros2/testing:good_test_converter_plugin.so",
    ],
    deps = [
        ":converter_plugin",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "default_converter_plugin.so",
    srcs = ["default_converter_plugin.cc"],
    linkshared = True,
    visibility = ["//visibility:public"],
    deps = [
        ":converter_plugin_helpers",
        ":converter_plugin_types",
        ":detection_from_ros2",
        ":header_from_ros2",
        ":navsat_from_ros2",
        ":odometry_from_ros2",
        ":oriented_box_from_ros2",
        ":pose_from_ros2",
        ":primitives_from_ros2",
        ":time_from_ros2",
        ":transform_from_ros2",
        "//resim/assert",
    ],
)

cc_library(
    name = "converter_plugin_helpers",
    hdrs = ["converter_plugin_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":converter_plugin_types",
        "//resim/assert",
        "//resim/utils/proto:dependency_file_descriptor_set",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_test(
    name = "default_converter_plugin_test",
    srcs = ["default_converter_plugin_test.cc"],
    data = [":default_converter_plugin.so"],
    deps = [
        ":converter_plugin",
        ":detection_from_ros2",
        ":header_from_ros2",
        ":navsat_from_ros2",
        ":odometry_from_ros2",
        ":oriented_box_from_ros2",
        ":pose_from_ros2",
        ":primitives_from_ros2",
        ":time_from_ros2",
        ":transform_from_ros2",
        "//resim/assert",
        "//resim/msg:fuzz_helpers",
        "//resim/msg:odometry_proto_cc",
        "//resim/msg:transform_proto_cc",
        "//resim/utils:inout",
        "//resim/utils/proto:dependency_file_descriptor_set",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "resim_log_from_ros2",
    srcs = ["resim_log_from_ros2.cc"],
    hdrs = ["resim_log_from_ros2.hh"],
    deps = [
        ":converter_plugin",
        "//resim/assert",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:inout",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rosbag2//:rosbag2_cpp",
    ],
)

cc_library(
    name = "resim_log_from_ros2_test_helpers",
    testonly = True,
    srcs = ["resim_log_from_ros2_test_helpers.cc"],
    hdrs = ["resim_log_from_ros2_test_helpers.hh"],
    deps = [
        ":odometry_from_ros2",
        ":transform_from_ros2",
        "//resim/converter:fuzz_helpers",
        "//resim/msg:fuzz_helpers",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/time:timestamp",
        "//resim/utils:inout",
        "//resim/utils:tuple_utils",
        "//resim/utils/proto:dependency_file_descriptor_set",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rosbag2//:rosbag2_cpp",
        "@ros2_rosbag2//:rosbag2_storage",
    ],
)

ros2_cpp_test(
    name = "resim_log_from_ros2_test",
    testonly = True,  # Needed because ros2_cpp_test generates a non-test rule
    srcs = ["resim_log_from_ros2_test.cc"],
    data = ["//resim/ros2/testing:resim_log_from_ros2_test_converter_plugin.so"],
    set_up_ament = True,
    deps = [
        ":resim_log_from_ros2",
        ":resim_log_from_ros2_test_helpers",
        "//resim/testing:test_directory",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

ros2_cpp_binary(
    name = "convert_log",
    srcs = ["convert_log.cc"],
    data = [":default_converter_plugin.so"],
    set_up_ament = True,
    visibility = ["//visibility:public"],
    deps = [
        ":converter_plugin",
        ":resim_log_from_ros2",
        "//resim/assert",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:inout",
        "@cxxopts",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rosbag2//:rosbag2_cpp",
    ],
)

pybind_extension(
    name = "resim_log_from_ros2_test_helpers_python",
    testonly = True,
    srcs = ["resim_log_from_ros2_test_helpers_python.cc"],
    deps = [
        ":resim_log_from_ros2_test_helpers",
    ],
)

pybind_extension(
    name = "resim_log_from_ros2_python",
    srcs = ["resim_log_from_ros2_python.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":resim_log_from_ros2",
    ],
)

ros2_ament_setup(
    name = "resim_log_from_ros2_python_test_ament",
    testonly = True,
    deps = [
        ":default_converter_plugin.so",
        ":resim_log_from_ros2_python.so",
        ":resim_log_from_ros2_test_helpers_python.so",
    ],
)

py_test(
    name = "resim_log_from_ros2_python_test",
    srcs = ["resim_log_from_ros2_python_test.py"],
    data = [
        ":resim_log_from_ros2_python.so",
        ":resim_log_from_ros2_python_test_ament",
        ":resim_log_from_ros2_test_helpers_python.so",
        "//resim/ros2/testing:resim_log_from_ros2_test_converter_plugin.so",
    ],
)

INIT_CONTENT = """# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os
import pathlib

RESIM_DIR = pathlib.Path(__file__).parent.parent.resolve()
LIB_DIR = pathlib.Path(__file__).parent.parent.parent.resolve() / "resim.libs"
os.environ["AMENT_PREFIX_PATH"] = str(LIB_DIR / "ament")
os.environ["RMW_IMPLEMENTATION"] = str(LIB_DIR / "librmw_cyclonedds.so")
"""

ros2_py_package(
    name = "ros2_py_package",
    initcontent = INIT_CONTENT,
    subpackage_prefix = "resim/ros2",
    visibility = ["//visibility:public"],
    deps = [
        ":default_converter_plugin.so",
        ":resim_log_from_ros2_python.so",
    ],
)

pkg_files(
    name = "ros2_py_package_stubs",
    srcs = [
        "//resim/ros2:resim_log_from_ros2_python.stubs",
    ],
    strip_prefix = ".",
    visibility = ["//visibility:public"],
)
