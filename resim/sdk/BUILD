load("@resim_python_client_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")
load("//resim/sdk/client:openapi_client.bzl", "openapi_client")

py_library(
    name = "sdk",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk/auth",
        "//resim/sdk/client",
        "//resim/sdk/metrics",
    ],
)

exports_files(["requirements.txt"])

## Python API Client
openapi_client(
    name = "client-gen",
    package_name = "client",
    config = "//resim/sdk/client:openapi_python_client_config.yaml",
    specification = "//resim/sdk/client:openapi.yaml",
)

py_library(
    name = "client",
    data = [":client-gen"],
    imports = ["client-gen"],
    visibility = ["//visibility:public"],
    deps = [
        requirement("httpx"),
        requirement("attrs"),
        requirement("python-dateutil"),
    ],
)
