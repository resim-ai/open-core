# rules_resim

This repository contains rules for creating and running ReSim builds based on oci images, and a
repository rule that makes it easy to run the CLI from bazel. Please refer to the following
documentation for details:

 - [Rule documentation](./rules.md)
 - [Repository rule documentation](./extensions.md)

Also please refer to the `examples` directory for a simple example of creating a build and a
multi-container build.

## Updating the Docs
To update the auto-generated docs with stardoc:

```bash
bazel run //:update_docs
```

## Using `rules_resim`

With `MODULE.bazel`:
```
bazel_dep(name = "rules_resim", version = "0.1.0")
archive_override(
    module_name = "rules_resim",
    sha256 = RULES_RESIM_CHECKSUM,
    strip_prefix = "open-core-{}/rules_resim".format(RULES_RESIM_VERSION),
    url = "https://github.com/resim-ai/open-core/archive/{}.zip".format(RULES_RESIM_VERSION),
)
```

With `WORKSPACE`:
```
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_resim",
    sha256 = RULES_RESIM_CHECKSUM,
    strip_prefix = "open-core-{}/rules_resim".format(RULES_RESIM_VERSION),
    url = "https://github.com/resim-ai/open-core/archive/{}.zip".format(RULES_RESIM_VERSION),
)

load("@rules_resim//:workspace_deps.bzl", "rules_resim_repositories")

rules_resim_repositories()
```

## Updating the CLI Versions and Sums

To update the checksums for the available versions, run the following script after ensuring any new
versions are added to it. This will update the `//private:cli_versions.json` file.
```bash
bazel run //scripts:generate_cli_checksums
```
