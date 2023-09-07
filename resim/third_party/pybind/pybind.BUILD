load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "pybind",
    hdrs = glob(["pybind11/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@python3_10//:python_headers",
    ],
)
