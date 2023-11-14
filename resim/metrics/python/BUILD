load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

py_library(
    name = "metrics",
    srcs = ["metrics.py"],
    deps = [
        requirement("numpy"),
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/utils/proto:uuid_proto_py",
        "@com_google_protobuf//:protobuf_python",
    ],
)

py_binary(
    name = "metrics_example",
    srcs = ["metrics_example.py"],
    deps = [
        requirement("numpy"),
        ":metrics",
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/utils/proto:uuid_proto_py",
        "@com_google_protobuf//:protobuf_python",
    ],
)

py_binary(
    name = "metrics_sanitycheck",
    srcs = ["metrics_sanitycheck.py"],
    deps = [
        ":metrics",
    ],
)
