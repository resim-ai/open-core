load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "rank",
    srcs = ["rank.py"],
    deps = [
        "//resim/metrics/python:emissions",
        "//resim/sdk",
    ],
)
