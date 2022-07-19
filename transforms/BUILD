load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "frame",
    hdrs = ["frame.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup",
        "//utils:uuid",
    ],
)

cc_test(
    name = "frame_test",
    srcs = ["frame_test.cc"],
    deps = [
        ":frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "framed_group",
    srcs = ["framed_group.cc"],
    hdrs = ["framed_group.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame",
        ":liegroup",
        ":liegroup_test_helpers",
        ":se3",
        ":so3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "framed_group_test",
    srcs = ["framed_group_test.cc"],
    deps = [
        ":frame",
        ":framed_group",
        ":se3",
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "liegroup",
    hdrs = ["liegroup.hh"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "liegroup_exp_diff",
    srcs = ["liegroup_exp_diff.cc"],
    hdrs = ["liegroup_exp_diff.hh"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "liegroup_exp_diff_test",
    srcs = ["liegroup_exp_diff_test.cc"],
    deps = [
        ":liegroup_exp_diff",
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "liegroup_test_helpers",
    srcs = ["liegroup_test_helpers.cc"],
    hdrs = ["liegroup_test_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":se3",
        ":so3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "liegroup_test_helpers_test",
    srcs = ["liegroup_test_helpers_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":se3",
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "liegroup_test",
    srcs = ["liegroup_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":se3",
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "se3",
    srcs = ["se3.cc"],
    hdrs = ["se3.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup",
        ":liegroup_exp_diff",
        ":so3",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "se3_test",
    srcs = ["se3_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":se3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "so3",
    srcs = ["so3.cc"],
    hdrs = ["so3.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "so3_test",
    srcs = ["so3_test.cc"],
    deps = [
        ":liegroup_test_helpers",
        ":so3",
        "@com_google_googletest//:gtest_main",
        "@libeigen//:eigen",
    ],
)
