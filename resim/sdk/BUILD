load("@rules_python//python:defs.bzl", "py_library", "py_binary")

py_library(
    name = "sdk",
    srcs = ["sdk.py", "sdk_helpers.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim-python-client",
        "//resim/auth/python:device_code_client",
        "//resim/auth/python:username_password_client",
        "//resim/metrics/python:emissions",
        "//resim/metrics:fetch_all_pages",
    ],
)

exports_files(["requirements.txt"])

py_binary(
    name = "test_sdk",
    srcs = ["test_sdk.py"],
    deps = [":sdk"],
)
