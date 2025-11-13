load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "sdk",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk/auth",
        "//resim/sdk/metrics",
    ],
)

exports_files(["requirements.txt"])
