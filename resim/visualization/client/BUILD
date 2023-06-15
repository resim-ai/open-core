load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "view_client",
    srcs = ["view_client.cc"],
    hdrs = ["view_client.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/auth:auth_client_interface",
        "//resim/auth:device_code_client",
        "//resim/utils:status",
        "//resim/visualization:view_client_interface",
        "//resim/visualization/client/proto:view_client_proto_cc",
        "//resim/visualization/proto:view_update_proto_cc",
        "//resim/visualization/proto:view_update_to_proto",
        "@com_github_google_glog//:glog",
        "@cpr",
        "@fmt",
    ],
)
