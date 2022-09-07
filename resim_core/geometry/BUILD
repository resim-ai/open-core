load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

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
