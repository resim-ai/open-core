load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "oriented_box",
    srcs = ["oriented_box.cc"],
    hdrs = ["oriented_box.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "oriented_box_test",
    srcs = ["oriented_box_test.cc"],
    deps = [
        ":oriented_box",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gjk_distance_subalgorithm",
    srcs = ["gjk_distance_subalgorithm.cc"],
    hdrs = ["gjk_distance_subalgorithm.hh"],
    deps = [
        "//resim_core/utils:inout",
        "//resim_core/utils:integer_power",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "gjk_distance_subalgorithm_test",
    srcs = ["gjk_distance_subalgorithm_test.cc"],
    deps = [
        ":gjk_distance_subalgorithm",
        "//resim_core/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gjk_algorithm",
    srcs = ["gjk_algorithm.cc"],
    hdrs = ["gjk_algorithm.hh"],
    deps = [
        ":gjk_distance_subalgorithm",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "gjk_algorithm_test",
    srcs = ["gjk_algorithm_test.cc"],
    deps = [
        ":gjk_algorithm",
        ":oriented_box",
        "//resim_core/testing:random_matrix",
        "//resim_core/transforms:liegroup_concepts",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "boxes_collide",
    srcs = ["boxes_collide.cc"],
    hdrs = ["boxes_collide.hh"],
    deps = [
        ":gjk_algorithm",
        ":oriented_box",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "boxes_collide_test",
    srcs = ["boxes_collide_test.cc"],
    deps = [
        ":boxes_collide",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:se3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "wireframe",
    srcs = ["wireframe.cc"],
    hdrs = ["wireframe.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "wireframe_test",
    srcs = ["wireframe_test.cc"],
    deps = [
        ":wireframe",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "drone_wireframe",
    srcs = ["drone_wireframe.cc"],
    hdrs = ["drone_wireframe.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":wireframe",
    ],
)

cc_test(
    name = "drone_wireframe_test",
    srcs = ["drone_wireframe_test.cc"],
    deps = [
        ":drone_wireframe",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "bounding_box_from_wireframe",
    srcs = ["bounding_box_from_wireframe.cc"],
    hdrs = ["bounding_box_from_wireframe.hh"],
    deps = [
        ":oriented_box",
        ":wireframe",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "bounding_box_from_wireframe_test",
    srcs = ["bounding_box_from_wireframe_test.cc"],
    deps = [
        ":bounding_box_from_wireframe",
        "//resim_core/testing:random_matrix",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
