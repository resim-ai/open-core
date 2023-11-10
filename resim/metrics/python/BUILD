load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

py_library(
    name = "metrics",
    srcs = ["metrics.py"],
    deps = [
        ":metrics_utils",
        "//resim/metrics/proto:metrics_proto_py",
    ],
)

py_library(
    name = "metrics_writer",
    srcs = ["metrics_writer.py"],
    deps = [
        requirement("numpy"),
        ":metrics",
        ":metrics_utils",
        "//resim/metrics/proto:metrics_proto_py",
        "@com_google_protobuf//:protobuf_python",
    ],
)

py_library(
    name = "metrics_utils",
    srcs = ["metrics_utils.py"],
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
        ":metrics_utils",
        ":metrics_writer",
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/utils/proto:uuid_proto_py",
        "@com_google_protobuf//:protobuf_python",
    ],
)

py_test(
    name = "metrics_test",
    srcs = ["metrics_test.py"],
    deps = [
        requirement("numpy"),
        ":metrics",
        ":metrics_writer",
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/utils/proto:uuid_proto_py",
    ],
)
