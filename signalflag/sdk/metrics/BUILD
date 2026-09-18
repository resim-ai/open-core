load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "emissions",
    srcs = [
        "__init__.py",
        "emissions.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        requirement("numpy"),
        requirement("pyyaml"),
    ],
)

py_test(
    name = "emissions_test",
    srcs = ["emissions_test.py"],
    deps = [
        ":emissions",
    ],
)

py_library(
    name = "metrics",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":emissions",
    ],
)
