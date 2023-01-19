load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "test_directory",
    testonly = True,
    srcs = ["test_directory.cc"],
    hdrs = ["test_directory.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/utils:uuid",
    ],
)

cc_test(
    name = "test_directory_test",
    srcs = ["test_directory_test.cc"],
    deps = [
        ":test_directory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "random_matrix",
    testonly = True,
    hdrs = ["random_matrix.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "random_matrix_test",
    srcs = ["random_matrix_test.cc"],
    deps = [
        ":random_matrix",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
