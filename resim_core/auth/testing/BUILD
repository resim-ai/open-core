load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "mock_auth_server",
    testonly = 1,
    srcs = ["mock_auth_server.cc"],
    hdrs = ["mock_auth_server.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/assert",
        "//resim_core/auth/proto:auth_messages_proto_cc",
        "//resim_core/testing:mock_server",
        "//resim_core/utils:http_response",
        "@com_google_protobuf//:protobuf",
    ],
)
