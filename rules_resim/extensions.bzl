# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
# ReSim Extensions
"""

load("@rules_resim//private:cli.bzl", "resolve_cli_version", _resim_cli = "resim_cli")

resim_cli = _resim_cli

_versions = tag_class(
    doc = """Tag class to specify the versions of fetched tools.""",
    attrs = {
        "cli_version": attr.string(mandatory = False, doc = "The CLI release version (e.g. v0.29.0)"),
    },
)

def _extension_impl(module_ctx):
    cli_version = resolve_cli_version(module_ctx)
    platforms_by_version = json.decode(module_ctx.read(module_ctx.path(Label("//private:cli_versions.json"))))
    platforms = platforms_by_version[cli_version]

    resim_cli(name = "resim_cli", version = cli_version, platforms = platforms)

resim_cli_extension = module_extension(
    implementation = _extension_impl,
    tag_classes = {
        "versions": _versions,
    },
    doc = """Extension for pulling in the resim cli for multiple platforms.

For example, if you use this rule in your `MODULE.bazel`:
```
resim_cli = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
use_repo(resim_cli, "resim_cli")
```
You can run the CLI like so.
```
bazel run @resim_cli//:resim
```""",
)
