# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@aspect_bazel_lib//lib:tar.bzl", "tar")
load("@rules_oci//oci:defs.bzl", "oci_image", "oci_push", "oci_tarball")

tar(
    name = "entrypoint_layer",
    srcs = [
        "//resim/metrics:default_report_metrics",
    ],
)

oci_image(
    name = "default_report_metrics_build",
    base = "@python_image",
    entrypoint = ["resim/metrics/default_report_metrics"],
    tars = [":entrypoint_layer"],
    workdir = "/resim/metrics/default_report_metrics.runfiles/_main",
)

oci_tarball(
    name = "default_report_metrics_build_tarball",
    image = ":default_report_metrics_build",
    repo_tags = ["default-report-metrics-build:latest"],
)

oci_push(
    name = "default_report_metrics_build_push",
    image = ":default_report_metrics_build",
    repository = "public.ecr.aws/resim/open-metrics-builds",
)
