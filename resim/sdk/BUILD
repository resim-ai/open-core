load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "sdk",
    srcs = ["sdk.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim-python-client",
        "//resim/metrics/python:emissions",
    ],
)

exports_files(["requirements.txt"])
