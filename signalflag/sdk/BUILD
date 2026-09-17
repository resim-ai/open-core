load("@resim_python_client_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("//bazel:openapi_client.bzl", "openapi_client")

py_library(
    name = "sdk",
    srcs = [
        "__init__.py",
        "batch.py",
        "test.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//signalflag/sdk:client",
        "//signalflag/sdk/auth",
        "//signalflag/sdk/bff_client",
        "//signalflag/sdk/metrics",
    ],
)

py_test(
    name = "sdk_test",
    size = "small",
    srcs = [
        "batch_test.py",
        "sdk_test.py",
        "test_test.py",
    ],
    deps = [
        ":sdk",
    ],
)

## Python API Client
# Ideally, this would live in the client folder, but otherwise we end up with a bunch of horrid nested folders
openapi_client(
    name = "client-gen",
    package_name = "client",
    config = "//signalflag/sdk/client_generator:openapi_python_client_config.yaml",
    specification = "//signalflag/sdk/client_generator:customerapi.yaml",
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
        "//signalflag/sdk/client_generator:requirements.txt",
        ":requirements.txt",
    ],
    outs = ["combined_sdk_requirements.txt"],
    cmd = "cat $(SRCS) > $@",
    visibility = ["//visibility:public"],
)
