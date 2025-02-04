load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "update_open_core",
    srcs = ["update_open_core.py"],
    deps = [
        requirement("httpx"),
    ],
)
