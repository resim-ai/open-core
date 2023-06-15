load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "oscillator",
    testonly = True,
    srcs = ["oscillator.cc"],
    hdrs = ["oscillator.hh"],
    visibility = ["//resim/dynamics:__subpackages__"],
    deps = [
        "//resim/dynamics",
        "//resim/dynamics:controller",
        "//resim/time:timestamp",
        "//resim/utils:nullable_reference",
        "@libeigen//:eigen",
    ],
)
