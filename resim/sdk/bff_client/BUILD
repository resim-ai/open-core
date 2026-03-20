load("@resim_python_client_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "metrics",
    srcs = ["metrics.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk:client",
        "//resim/sdk/metrics:emissions",
    ],
)

py_test(
    name = "metrics_test",
    srcs = ["metrics_test.py"],
    deps = [
        ":metrics",
        requirement("httpx"),
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
