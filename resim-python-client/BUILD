load("@resim_python_client_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")
load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")
load(":openapi_client.bzl", "openapi_client")

exports_files(["requirements.txt"])

py_console_script_binary(
    name = "openapi-python-client",
    pkg = "@resim_python_client_dev_deps//openapi_python_client",
)

openapi_client(
    name = "resim-python-client-gen",
    config = "openapi_python_client_config.yaml",
    specification = "@resim_api_spec",
)

py_library(
    name = "resim-python-client",
    data = [":resim-python-client-gen"],
    imports = ["resim-python-client-gen"],
    visibility = ["//visibility:public"],
    deps = [
        requirement("httpx"),
        requirement("attrs"),
        requirement("python-dateutil"),
    ],
)
