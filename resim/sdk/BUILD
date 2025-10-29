load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "sdk",
    srcs = ["sdk.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim-python-client",
        "//resim/auth/python:device_code_client",
        "//resim/auth/python:username_password_client",
        "//resim/metrics/python:emissions",
    ],
)

exports_files(["requirements.txt"])
