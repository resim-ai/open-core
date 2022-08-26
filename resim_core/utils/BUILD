load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.hh"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "uuid_test",
    srcs = ["uuid_test.cc"],
    deps = [
        ":uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inout",
    hdrs = ["inout.hh"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "inout_test",
    srcs = ["inout_test.cc"],
    deps = [
        ":inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "random_vector",
    testonly = True,
    hdrs = ["random_vector.hh"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_test(
    name = "random_vector_test",
    srcs = ["random_vector_test.cc"],
    deps = [
        ":random_vector",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
