<!-- Generated with Stardoc: http://skydoc.bazel.build -->

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
for more details.

<a id="resim_build"></a>

## resim_build

<pre>
load("@rules_resim//:defs.bzl", "resim_build")

resim_build(<a href="#resim_build-name">name</a>, <a href="#resim_build-data">data</a>, <a href="#resim_build-auto_create_branch">auto_create_branch</a>, <a href="#resim_build-branch">branch</a>, <a href="#resim_build-build_spec">build_spec</a>, <a href="#resim_build-build_spec_env">build_spec_env</a>, <a href="#resim_build-description">description</a>,
            <a href="#resim_build-image_pushes">image_pushes</a>, <a href="#resim_build-project">project</a>, <a href="#resim_build-resim_name">resim_name</a>, <a href="#resim_build-system">system</a>, <a href="#resim_build-version">version</a>)
</pre>

This rule creates a single or multi-container ReSim build when run. By default, version
and branch information is gleaned from a workspace status command (through the
`STABLE_RESIM_VERSION` and `STABLE_RESIM_BRANCH` variables) if it is employed. Otherwise, these
values can be overridden using the attributes of this rule. If neither is employed, this target will
fail to build.

**ATTRIBUTES**


| Name  | Description | Type | Mandatory | Default |
| :------------- | :------------- | :------------- | :------------- | :------------- |
| <a id="resim_build-name"></a>name |  A unique name for this target.   | <a href="https://bazel.build/concepts/labels#target-names">Name</a> | required |  |
| <a id="resim_build-data"></a>data |  List of data runfiles   | <a href="https://bazel.build/concepts/labels">List of labels</a> | optional |  `[]`  |
| <a id="resim_build-auto_create_branch"></a>auto_create_branch |  Whether to automatically create branch if it doesn't exist   | Boolean | optional |  `True`  |
| <a id="resim_build-branch"></a>branch |  The name or ID of the branch to nest the build in, usually the associated git branch. Will override workspace status setting as described above.   | String | optional |  `""`  |
| <a id="resim_build-build_spec"></a>build_spec |  Paths to main compose file (may use extends)   | <a href="https://bazel.build/concepts/labels">Label</a> | optional |  `None`  |
| <a id="resim_build-build_spec_env"></a>build_spec_env |  Environment variables to set for the build_spec   | <a href="https://bazel.build/rules/lib/dict">Dictionary: String -> String</a> | optional |  `{}`  |
| <a id="resim_build-description"></a>description |  The description of the build, often a commit message (can include markdown). For backwards compatibility reasons, if name is omitted, the description will be used   | String | required |  |
| <a id="resim_build-image_pushes"></a>image_pushes |  List of oci image pushes   | <a href="https://bazel.build/concepts/labels">List of labels</a> | optional |  `[]`  |
| <a id="resim_build-project"></a>project |  The name or ID of the project to create the build in   | String | required |  |
| <a id="resim_build-resim_name"></a>resim_name |  The name of the build   | String | optional |  `""`  |
| <a id="resim_build-system"></a>system |  The name or ID of the system the build is an instance of   | String | required |  |
| <a id="resim_build-version"></a>version |  The version of the build image, usually a commit ID. Will override workspace status stting as described above.   | String | optional |  `""`  |


