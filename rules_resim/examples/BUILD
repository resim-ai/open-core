load("@rules_oci//oci:defs.bzl", "oci_image", "oci_push", "oci_load")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@resim_version//:defs.bzl", version = "RESIM_VERSION", branch = "RESIM_BRANCH")
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
    repository = "909785973729.dkr.ecr.us-east-1.amazonaws.com/customer-test-images",
    remote_tags = ["simple_build_{}".format(version)]
)

resim_build(
    name = "simple_build",
    project = "ReSim One Project",
    system = "ReSim One System",
    version = version,
    branch = branch,
    description = "A simple build which greets the runner.",
    resim_name = "Hello Build",
    image_pushes = [
        ":simple_build_push",
    ],
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
    tars = [":multicontainer_build_entrypoint_layer"],
    workdir = "/multicontainer_build_entrypoint.runfiles/_main",
    env = {"MCB_ROLE": "SENDER"},
)
oci_load(
    name = "multicontainer_build_sender_load",
    image = ":multicontainer_build_sender_image",
    repo_tags = ["multicontainer-build-sender:latest"],
)

oci_push(
    name = "multicontainer_build_sender_push",
    image = ":multicontainer_build_sender_image",
    repository = "909785973729.dkr.ecr.us-east-1.amazonaws.com/customer-test-images",
    remote_tags = ["mcb_sender_{}".format(version)]
)

# Receiver image
oci_image(
    name = "multicontainer_build_receiver_image",
    base = "@alpine",
    entrypoint = ["./multicontainer_build_entrypoint"],
    tars = [":multicontainer_build_entrypoint_layer"],
    workdir = "/multicontainer_build_entrypoint.runfiles/_main",
    env = {"MCB_ROLE": "RECEIVER"},
)
oci_load(
    name = "multicontainer_build_receiver_load",
    image = ":multicontainer_build_receiver_image",
    repo_tags = ["multicontainer-build-receiver:latest"],
)

oci_push(
    name = "multicontainer_build_receiver_push",
    image = ":multicontainer_build_receiver_image",
    repository = "909785973729.dkr.ecr.us-east-1.amazonaws.com/customer-test-images",
    remote_tags = ["mcb_receiver_{}".format(version)]
)

resim_build(
    name = "mcb_build",
    project = "ReSim One Project",
    system = "ReSim One System",
    version = version,
    branch = branch,
    build_spec = "docker-compose.yml",
    description = "A simple multicontainer build which greets the runner.",
    resim_name = "Hello Build",
    image_pushes = [
        ":multicontainer_build_receiver_push",
        ":multicontainer_build_sender_push",        
    ],
    data = [
        "docker-compose.base.yml",
    ],
    build_spec_env = {
        "RESIM_VERSION": version
    },
)

