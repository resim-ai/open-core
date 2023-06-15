load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "dynamics",
    hdrs = ["dynamics.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/time:timestamp",
        "//resim/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "controller",
    hdrs = ["controller.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//resim/time:timestamp",
        "//resim/utils:nullable_reference",
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
        "//resim/time:timestamp",
        "//resim/utils:nullable_reference",
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
        "//resim/dynamics/testing:oscillator",
        "//resim/time:timestamp",
        "//resim/utils:nullable_reference",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "forward_euler",
    hdrs = ["forward_euler.hh"],
    visibility = ["//resim/dynamics:__subpackages__"],
    deps = [
        ":dynamics",
        ":integrator",
        "//resim/time:timestamp",
    ],
)

cc_test(
    name = "forward_euler_test",
    srcs = ["forward_euler_test.cc"],
    deps = [
        ":controller",
        ":dynamics",
        ":forward_euler",
        "//resim/dynamics/testing:oscillator",
        "//resim/time:sample_interval",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.hh"],
    visibility = ["//resim/dynamics:__subpackages__"],
    deps = [
        "@au//au",
    ],
)
