load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "mock_device_code_server",
    testonly = 1,
    srcs = ["mock_device_code_server.cc"],
    hdrs = ["mock_device_code_server.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/assert",
        "//resim_core/auth/proto:auth_messages_proto_cc",
        "//resim_core/testing:mock_server",
        "//resim_core/utils:http_response",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "mock_client_credentials_server",
    testonly = 1,
    srcs = ["mock_client_credentials_server.cc"],
    hdrs = ["mock_client_credentials_server.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/assert",
        "//resim_core/auth/proto:auth_messages_proto_cc",
        "//resim_core/testing:mock_server",
        "//resim_core/utils:http_response",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "mock_auth_client",
    testonly = 1,
    hdrs = ["mock_auth_client.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/auth:auth_client_interface",
    ],
)

cc_test(
    name = "mock_auth_client_test",
    srcs = ["mock_auth_client_test.cc"],
    deps = [
        ":mock_auth_client",
        "@com_google_googletest//:gtest_main",
    ],
)
