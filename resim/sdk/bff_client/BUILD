load("@resim_python_client_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "metrics",
    srcs = ["metrics.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk:client",
    ],
)

py_library(
    name = "bff_client",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":metrics",
        requirement("httpx"),
    ],
)
