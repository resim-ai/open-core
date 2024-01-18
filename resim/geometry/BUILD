# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "oriented_box",
    srcs = ["oriented_box.cc"],
    hdrs = ["oriented_box.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/assert",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "oriented_box_test",
    srcs = ["oriented_box_test.cc"],
    deps = [
        ":oriented_box",
        "//resim/assert",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gjk_distance_subalgorithm",
    srcs = ["gjk_distance_subalgorithm.cc"],
    hdrs = ["gjk_distance_subalgorithm.hh"],
    deps = [
        "//resim/assert",
        "//resim/utils:inout",
        "//resim/utils:integer_power",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "gjk_distance_subalgorithm_test",
    srcs = ["gjk_distance_subalgorithm_test.cc"],
    deps = [
        ":gjk_distance_subalgorithm",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gjk_algorithm",
    srcs = ["gjk_algorithm.cc"],
    hdrs = ["gjk_algorithm.hh"],
    deps = [
        ":gjk_distance_subalgorithm",
        "//resim/assert",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "gjk_algorithm_test",
    srcs = ["gjk_algorithm_test.cc"],
    deps = [
        ":gjk_algorithm",
        ":oriented_box",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "//resim/transforms:liegroup_concepts",
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
        "//resim/assert",
        "//resim/transforms:liegroup_concepts",
        "//resim/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "boxes_collide_test",
    srcs = ["boxes_collide_test.cc"],
    deps = [
        ":boxes_collide",
        "//resim/assert",
        "//resim/transforms:se3",
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
        "//resim/assert",
    ],
)

cc_test(
    name = "drone_wireframe_test",
    srcs = ["drone_wireframe_test.cc"],
    deps = [
        ":drone_wireframe",
        "//resim/assert",
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
        "//resim/assert",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "bounding_box_from_wireframe_test",
    srcs = ["bounding_box_from_wireframe_test.cc"],
    deps = [
        ":bounding_box_from_wireframe",
        "//resim/assert",
        "//resim/testing:random_matrix",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "polygon_utils",
    srcs = ["polygon_utils.cc"],
    hdrs = ["polygon_utils.hh"],
    deps = [
        "//resim/assert",
        "//resim/math:clamp",
        "//resim/testing:random_matrix",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "polygon_utils_test",
    srcs = ["polygon_utils_test.cc"],
    deps = [
        ":polygon_utils",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "polygon_distance",
    srcs = ["polygon_distance.cc"],
    hdrs = ["polygon_distance.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":gjk_algorithm",
        ":polygon_utils",
        "//resim/assert",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "polygon_distance_test",
    srcs = ["polygon_distance_test.cc"],
    deps = [
        ":polygon_distance",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
