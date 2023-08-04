load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "test_helpers",
    testonly = True,
    srcs = ["test_helpers.cc"],
    hdrs = ["test_helpers.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/transforms:se3",
        "//resim/utils:match",
        "//resim/visualization:view_primitive",
    ],
)

cc_test(
    name = "test_helpers_test",
    srcs = ["test_helpers_test.cc"],
    deps = [
        ":test_helpers",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/utils:uuid",
        "//resim/visualization:view_primitive",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "mock_server",
    testonly = True,
    srcs = ["mock_server.cc"],
    hdrs = ["mock_server.hh"],
    visibility = ["//resim/visualization:__subpackages__"],
    deps = [
        "//resim/testing:mock_server",
        "//resim/utils:http_response",
        "//resim/utils:uuid",
        "//resim/visualization:view_update",
        "//resim/visualization/client/proto:view_client_proto_cc",
        "//resim/visualization/proto:view_update_proto_cc",
        "//resim/visualization/proto:view_update_to_proto",
        "@com_github_google_glog//:glog",
        "@fmt",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "mock_server_test",
    srcs = ["mock_server_test.cc"],
    deps = [
        ":mock_server",
        ":test_helpers",
        "//resim/transforms:se3",
        "//resim/utils:http_response",
        "//resim/utils:uuid",
        "//resim/visualization:view_primitive",
        "//resim/visualization:view_update",
        "@com_google_googletest//:gtest_main",
        "@cpr",
        "@fmt",
    ],
)
