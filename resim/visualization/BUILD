load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "color",
    hdrs = ["color.hh"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "view_primitive",
    hdrs = ["view_primitive.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/actor/state:trajectory",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:uuid",
    ],
)

cc_library(
    name = "view_update",
    hdrs = ["view_update.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_primitive",
    ],
)

cc_library(
    name = "view_client_interface",
    hdrs = ["view_client_interface.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_update",
        "//resim/utils:status",
    ],
)

cc_library(
    name = "view",
    srcs = ["view.cc"],
    hdrs = ["view.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":view_client_interface",
        ":view_primitive",
        ":view_update",
        "//resim/assert",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:status",
        "//resim/utils:uuid",
        "//resim/visualization/client:view_client",
        "@com_google_googletest//:gtest_prod",
    ],
)

cc_test(
    name = "view_test",
    srcs = ["view_test.cc"],
    deps = [
        ":view",
        ":view_client_interface",
        "//resim/actor/state:trajectory",
        "//resim/assert",
        "//resim/auth/testing:mock_auth_client",
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/curves:test_helpers",
        "//resim/testing:test_directory",
        "//resim/transforms:frame",
        "//resim/transforms:framed_vector",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:http_response",
        "//resim/utils:match",
        "//resim/utils:status",
        "//resim/visualization/client:view_client",
        "//resim/visualization/testing:mock_server",
        "//resim/visualization/view_server:view_server_test_helper",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)
