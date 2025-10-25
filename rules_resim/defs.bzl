# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
# ReSim Rules

These rules are designed to make it easy to create and run ReSim builds based on oci images created
and pushed via `rules_oci`. For instance, if you have an oci image you push with `rules_oci` like
so:

```
load("@rules_oci//oci:defs.bzl", "oci_push")

# ...

oci_push(
    name = "simple_build_push",
    image = ":simple_build_image",
    remote_tags = ["simple_build_{}".format(version)],
    repository = "public.ecr.aws/resim/core",
)
```

Then you can create a runnable target to create a single-container ReSim build for you like so:

```
resim_build(
    name = "simple_build",
    branch = branch,
    description = "A simple build which greets the runner.",
    image_pushes = [
        ":simple_build_push",
    ],
    project = "My Project Name",
    resim_name = "My Build Name",
    system = "My System Name,
    version = version,
)
```

This target will automatically create a resim build based off the image and (first) remote tag
configured in the `oci_push()` target. Builds can also be created for multicontainer builds with a
docker compose input. See [here](https://docs.resim.ai/guides/multi-container-builds/) for more
details on multi-container builds.

In order to utilize automatic git version and branch inference, you can set up a workspace status
command in your repo like so:

1. Add a `workspace_status.sh` script like so:
```
cat <<EOF
STABLE_RESIM_VERSION $(git rev-parse HEAD)
STABLE_RESIM_BRANCH $(git rev-parse --abbrev-ref HEAD)
EOF
```
2. Add the following to your `.bazelrc`:
```
build --workspace_status_command=$(pwd)/workspace_status.sh
```
Once this is complete, the `resim_build()` rule will infer the version and branch automatically and
utilize them for your resim build. See [here](https://bazel.build/docs/user-manual#workspace-status)
for more details."""

load("@rules_resim//private:resim_build.bzl", _resim_build = "resim_build")

resim_build = _resim_build
