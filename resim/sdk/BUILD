load("@resim_python_client_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:openapi_client.bzl", "openapi_client")

py_library(
    name = "sdk",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk:client",
        "//resim/sdk/auth",
        "//resim/sdk/metrics",
    ],
)

## Python API Client
# Ideally, this would live in the client folder, but otherwise we end up with a bunch of horrid nested folders
openapi_client(
    name = "client-gen",
    package_name = "client",
    config = "//resim/sdk/client_generator:openapi_python_client_config.yaml",
    specification = "//resim/sdk/client_generator:openapi.yaml",
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

genrule(
    name = "combined_requires",
    srcs = [
        "//resim/sdk/client_generator:requirements.txt",
        ":requirements.txt",
    ],
    outs = ["combined_sdk_requirements.txt"],
    cmd = "cat $(SRCS) > $@",
    visibility = ["//visibility:public"],
)
