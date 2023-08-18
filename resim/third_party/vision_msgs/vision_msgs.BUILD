load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "cpp_ros2_interface_library",
    "ros2_interface_library",
)

ros2_interface_library(
    name = "vision_msgs",
    srcs = glob(["*.msg"]),
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:std_msgs",
    ],
)

cpp_ros2_interface_library(
    name = "cpp_vision_msgs",
    visibility = ["//visibility:public"],
    deps = [":vision_msgs"],
)
