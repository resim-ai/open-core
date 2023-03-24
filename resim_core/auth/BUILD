load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "auth_client_interface",
    hdrs = ["auth_client_interface.hh"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "client_credentials_client",
    srcs = ["client_credentials_client.cc"],
    hdrs = ["client_credentials_client.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":auth_client_interface",
        "//resim_core/assert",
        "//resim_core/auth/proto:auth_messages_proto_cc",
        "@com_google_protobuf//:protobuf",
        "@httplib",
    ],
)

cc_test(
    name = "client_credentials_client_test",
    srcs = ["client_credentials_client_test.cc"],
    deps = [
        ":client_credentials_client",
        "//resim_core/auth/testing:mock_client_credentials_server",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "device_code_client",
    srcs = ["device_code_client.cc"],
    hdrs = ["device_code_client.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":auth_client_interface",
        "//resim_core/assert",
        "//resim_core/auth/proto:auth_messages_proto_cc",
        "//resim_core/utils:http_response",
        "//resim_core/utils:inout",
        "@com_google_protobuf//:protobuf",
        "@curl",
    ],
)

cc_test(
    name = "device_code_client_test",
    srcs = ["device_code_client_test.cc"],
    deps = [
        ":device_code_client",
        "//resim_core/assert",
        "//resim_core/auth/testing:mock_device_code_server",
        "//resim_core/testing:test_directory",
        "@com_google_googletest//:gtest_main",
    ],
)
