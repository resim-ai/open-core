load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

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
        "//resim/msg:fuzz_helpers",
        "//resim/msg:transform_proto_cc",
        "//resim/testing:fuzz_helpers",
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
        "//resim/msg:fuzz_helpers",
        "//resim/testing:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "odometry_from_ros2",
    srcs = ["odometry_from_ros2.cc"],
    hdrs = ["odometry_from_ros2.hh"],
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
        "//resim/msg:fuzz_helpers",
        "//resim/testing:fuzz_helpers",
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
        "//resim/geometry/proto:fuzz_helpers",
        "//resim/testing:fuzz_helpers",
        "//resim/transforms:frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "detection_from_ros2",
    srcs = ["detection_from_ros2.cc"],
    hdrs = ["detection_from_ros2.hh"],
    deps = [
        ":detection_proto_cc",
        ":header_from_ros2",
        ":oriented_box_from_ros2",
        "@vision_msgs//:cpp_vision_msgs",
    ],
)

cc_test(
    name = "detection_from_ros2_test",
    srcs = ["detection_from_ros2_test.cc"],
    deps = [
        ":detection_from_ros2",
        ":fuzz_helpers",
        "//resim/testing:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)
