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
        "//resim_core/utils:random_vector",
        "@com_google_googletest//:gtest_main",
    ],
)
