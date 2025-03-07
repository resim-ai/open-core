# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@aspect_bazel_lib//lib:tar.bzl", "tar")
load("@rules_oci//oci:defs.bzl", "oci_image", "oci_push")

tar(
    name = "entrypoint_layer",
    srcs = [
        "entrypoint.sh",
    ],
)

oci_image(
    name = "log_ingest_build",
    base = "@alpine",
    entrypoint = ["./entrypoint.sh"],
    tars = [":entrypoint_layer"],
    workdir = "/resim/builds/log-ingest/",
)

oci_push(
    name = "log_ingest_build_push",
    image = ":log_ingest_build",
    repository = "public.ecr.aws/resim/open-builds/log-ingest",
    tags = ["latest"],
)
