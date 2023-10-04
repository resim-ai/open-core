load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "metrics",
    srcs = ["metrics.py"],
    deps = [
        requirement("numpy"),
        "//resim/metrics/proto:metrics_proto_py",
    ],
)
