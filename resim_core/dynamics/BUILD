load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "dynamics",
    hdrs = ["dynamics.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/time:timestamp",
        "//resim_core/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "controller",
    hdrs = ["controller.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//resim_core/time:timestamp",
        "//resim_core/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "integrator",
    hdrs = ["integrator.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":controller",
        ":dynamics",
        "//resim_core/time:timestamp",
        "//resim_core/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "integrator_test",
    srcs = ["integrator_test.cc"],
    deps = [
        ":controller",
        ":dynamics",
        ":integrator",
        "//resim_core/dynamics/testing:oscillator",
        "//resim_core/time:timestamp",
        "//resim_core/utils:nullable_reference",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "forward_euler",
    hdrs = ["forward_euler.hh"],
    visibility = ["//resim_core/dynamics:__subpackages__"],
    deps = [
        ":dynamics",
        ":integrator",
        "//resim_core/time:timestamp",
    ],
)

cc_test(
    name = "forward_euler_test",
    srcs = ["forward_euler_test.cc"],
    deps = [
        ":controller",
        ":dynamics",
        ":forward_euler",
        "//resim_core/dynamics/testing:oscillator",
        "//resim_core/time:sample_interval",
        "@com_google_googletest//:gtest_main",
    ],
)
