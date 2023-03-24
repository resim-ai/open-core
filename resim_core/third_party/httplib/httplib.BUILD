load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "httplib",
    hdrs = ["httplib.h"],
    defines = [
        "CPPHTTPLIB_OPENSSL_SUPPORT",
    ],
    includes = [
        ".",
    ],
    linkopts = [
        "-lcrypto",
        "-lssl",
    ],
    visibility = ["//visibility:public"],
)
