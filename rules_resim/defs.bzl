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

Once you have a build, you can also run it against an existing resim test suite by defining a target
with the `resim_test_suite_run()` rule:

```
resim_test_suite_run(
    name = "simple_build_run",
    resim_build = ":simple_build",
    test_suite = "ReSim Test Suite",
)
```

The project is inferred from the build, and some other batch parameters
(e.g. `allowable_failure_percent` and `pool_labels`) are also configurable. With this, one can build
the images, push them, register them as a build with ReSim, and run them against a test suite with a
single comand:

```
bazel run //:simple_build_run # -- --other --flags --to --pass --to --the --resim --cli
```
"""

load("@rules_resim//private:resim_build.bzl", _resim_build = "resim_build")
load("@rules_resim//private:resim_test_suite.bzl", _resim_test_suite_run = "resim_test_suite_run")

resim_build = _resim_build
resim_test_suite_run = _resim_test_suite_run
