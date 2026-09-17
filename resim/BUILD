load("@rules_python//python:defs.bzl", "py_library", "py_test")

filegroup(
    name = "py_typed_marker",
    srcs = ["py.typed"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "alias",
    srcs = ["_alias.py"],
    visibility = ["//visibility:public"],
)

py_test(
    name = "compat_test",
    size = "small",
    srcs = ["compat_test.py"],
    deps = [
        "//resim/demo:compat",
        "//resim/sdk:compat",
    ],
)
