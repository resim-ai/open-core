load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "auth_client",
    srcs = ["auth_client.cc"],
    hdrs = ["auth_client.hh"],
    deps = [
        "//resim_core/assert",
        "//resim_core/auth/proto:auth_messages_proto_cc",
        "//resim_core/utils:http_response",
        "//resim_core/utils:inout",
        "@com_google_protobuf//:protobuf",
        "@curl",
    ],
)

cc_test(
    name = "auth_client_test",
    srcs = ["auth_client_test.cc"],
    deps = [
        ":auth_client",
        "//resim_core/assert",
        "//resim_core/auth/testing:mock_auth_server",
        "//resim_core/testing:test_directory",
        "@com_google_googletest//:gtest_main",
    ],
)
