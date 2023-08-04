load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "example_00_rigid_transform",
    srcs = ["example_00_rigid_transform.cc"],
    deps = [
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_01_framed_transform_composition",
    srcs = ["example_01_framed_transform_composition.cc"],
    deps = [
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_02_distance_parameterized_unit_circle",
    srcs = ["example_02_distance_parameterized_unit_circle.cc"],
    deps = [
        "//resim/curves:d_curve",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_03_time_parameterized_closed_curve",
    srcs = ["example_03_time_parameterized_closed_curve.cc"],
    deps = [
        "//resim/curves:t_curve",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_04_actor_trajectory",
    srcs = ["example_04_actor_trajectory.cc"],
    deps = [
        "//resim/actor/state:rigid_body_state",
        "//resim/actor/state:trajectory",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:uuid",
        "//resim/visualization:view",
    ],
)
