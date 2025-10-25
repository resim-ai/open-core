# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@aspect_bazel_lib//lib:expand_template.bzl", "expand_template")
load("@resim_version//:defs.bzl", branch = "RESIM_BRANCH", version = "RESIM_VERSION")
load("@rules_oci//oci:defs.bzl", "oci_image", "oci_load", "oci_push")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@rules_resim//:defs.bzl", "resim_build")

################################################################################
# Common

filegroup(
    name = "greeting",
    srcs = ["greeting.txt"],
)

################################################################################
# Simple (single container) build

sh_binary(
    name = "simple_build_entrypoint",
    srcs = ["simple_build_entrypoint.sh"],
    data = [":greeting"],
)

pkg_tar(
    name = "simple_build_entrypoint_layer",
    srcs = [
        ":simple_build_entrypoint",
    ],
    include_runfiles = True,
    strip_prefix = ".",
)

oci_image(
    name = "simple_build_image",
    base = "@alpine",
    entrypoint = ["./simple_build_entrypoint"],
    tars = [":simple_build_entrypoint_layer"],
    workdir = "/simple_build_entrypoint.runfiles/_main",
)

oci_load(
    name = "simple_build_load",
    image = ":simple_build_image",
    repo_tags = ["simple-build:latest"],
)

oci_push(
    name = "simple_build_push",
    image = ":simple_build_image",
    remote_tags = ["simple_build_{}".format(version)],
    repository = "public.ecr.aws/resim/core",
)

resim_build(
    name = "simple_build",
    branch = branch,
    description = "A simple build which greets the runner.",
    image_pushes = [
        ":simple_build_push",
    ],
    project = "ReSim One Project",
    resim_name = "Simple Build",
    system = "ReSim One System",
    version = version,
)

################################################################################
# Multi-container build

sh_binary(
    name = "multicontainer_build_entrypoint",
    srcs = ["multicontainer_build_entrypoint.sh"],
    data = [":greeting"],
)

pkg_tar(
    name = "multicontainer_build_entrypoint_layer",
    srcs = [
        ":multicontainer_build_entrypoint",
    ],
    include_runfiles = True,
    strip_prefix = ".",
)

# Sender image
oci_image(
    name = "multicontainer_build_sender_image",
    base = "@alpine",
    entrypoint = ["./multicontainer_build_entrypoint"],
    env = {"MCB_ROLE": "SENDER"},
    tars = [":multicontainer_build_entrypoint_layer"],
    workdir = "/multicontainer_build_entrypoint.runfiles/_main",
)

oci_load(
    name = "multicontainer_build_sender_load",
    image = ":multicontainer_build_sender_image",
    repo_tags = ["multicontainer-build-sender:latest"],
)

expand_template(
    name = "sender_tags",
    out = "_sender_tags.txt",
    stamp = 1,
    stamp_substitutions = {"0.0.0": "{{STABLE_RESIM_VERSION}}"},
    template = ["mcb_sender_0.0.0"],
)

oci_push(
    name = "multicontainer_build_sender_push",
    image = ":multicontainer_build_sender_image",
    remote_tags = ":sender_tags",
    repository = "public.ecr.aws/resim/core",
)

# Receiver image
oci_image(
    name = "multicontainer_build_receiver_image",
    base = "@alpine",
    entrypoint = ["./multicontainer_build_entrypoint"],
    env = {"MCB_ROLE": "RECEIVER"},
    tars = [":multicontainer_build_entrypoint_layer"],
    workdir = "/multicontainer_build_entrypoint.runfiles/_main",
)

oci_load(
    name = "multicontainer_build_receiver_load",
    image = ":multicontainer_build_receiver_image",
    repo_tags = ["multicontainer-build-receiver:latest"],
)

expand_template(
    name = "receiver_tags",
    out = "_receiver_tags.txt",
    stamp = 1,
    stamp_substitutions = {"0.0.0": "{{STABLE_RESIM_VERSION}}"},
    template = ["mcb_receiver_0.0.0"],
)

oci_push(
    name = "multicontainer_build_receiver_push",
    image = ":multicontainer_build_receiver_image",
    remote_tags = ":receiver_tags",
    repository = "public.ecr.aws/resim/core",
)

resim_build(
    name = "mcb_build",
    build_spec = "docker-compose.yml",
    build_spec_env = {
        "RESIM_VERSION": version,
    },
    data = [
        "docker-compose.base.yml",
    ],
    description = "A simple multicontainer build which greets the runner.",
    image_pushes = [
        ":multicontainer_build_receiver_push",
        ":multicontainer_build_sender_push",
    ],
    project = "ReSim One Project",
    resim_name = "Simple MCB Build",
    system = "ReSim One System",
)
