load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.hh"],
    deps = [
        "//resim_core/transforms:framed_group",
    ],
)

cc_test(
    name = "state_test",
    srcs = ["state_test.cc"],
    deps = [
        ":state",
        "//resim_core/testing:random_matrix",
        "//resim_core/transforms:framed_group",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inertia",
    srcs = ["inertia.cc"],
    hdrs = ["inertia.hh"],
    deps = [
        "//resim_core/transforms:framed_group",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "inertia_test",
    srcs = ["inertia_test.cc"],
    deps = [
        ":inertia",
        "//resim_core/testing:random_matrix",
        "//resim_core/transforms:framed_group",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "dynamics",
    srcs = ["dynamics.cc"],
    hdrs = ["dynamics.hh"],
    deps = [
        ":inertia",
        ":state",
        "//resim_core/assert",
        "//resim_core/dynamics",
        "//resim_core/time:timestamp",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "dynamics_test",
    srcs = ["dynamics_test.cc"],
    deps = [
        ":dynamics",
        ":inertia",
        ":state",
        "//resim_core/assert",
        "//resim_core/dynamics:forward_euler",
        "//resim_core/time:sample_interval",
        "//resim_core/time:timestamp",
        "//resim_core/transforms:liegroup_test_helpers",
        "//resim_core/utils:nullable_reference",
        "@com_google_googletest//:gtest_main",
    ],
)
