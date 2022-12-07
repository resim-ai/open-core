load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "dynamics",
    hdrs = ["dynamics.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/utils:nullable_reference",
    ],
)

cc_library(
    name = "cost_function",
    hdrs = ["cost_function.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/utils:nullable_reference",
    ],
)

cc_library(
    name = "ilqr",
    hdrs = ["ilqr.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":cost_function",
        ":dynamics",
        "//resim_core/utils:double_buffer",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "ilqr_test",
    size = "small",
    srcs = ["ilqr_test.cc"],
    deps = [
        ":ilqr",
        "//resim_core/utils:random_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)
