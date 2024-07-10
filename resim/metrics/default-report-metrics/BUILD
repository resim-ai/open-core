load("@aspect_bazel_lib//lib:tar.bzl", "tar")
load("@aspect_rules_py//py:defs.bzl", "py_binary")
load("@rules_oci//oci:defs.bzl", "oci_image", "oci_push", "oci_tarball")

py_binary(
    name = "entrypoint",
    srcs = ["entrypoint.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/metrics:default_report_metrics",
    ],
)

tar(
    name = "entrypoint_layer",
    srcs = [
        "//src:entrypoint",
    ],
)

oci_image(
    name = "default_report_metrics_build",
    base = "@ubuntu",
    entrypoint = ["/src/entrypoint"],
    tars = [":entrypoint_layer"],
)

oci_tarball(
    name = "default_report_metrics_build_tarball",
    image = ":default_report_metrics_build",
    repo_tags = ["default-report-metrics-build:latest"],
)

oci_push(
    name = "default_report_metrics_build_push",
    image = ":default_report_metrics_build",
    repository = "909785973729.dkr.ecr.us-east-1.amazonaws.com/customer-test-images",  # Test!!
)
