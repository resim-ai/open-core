# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Repository rule to get the version of open core passed into the release workflow."""

def _resim_version_impl(repository_ctx):
    version = repository_ctx.os.environ.get("RESIM_VERSION", default = "0.0.0")
    if version.startswith("v"):
        version = version[1:]
    branch = repository_ctx.os.environ.get("RESIM_BRANCH", default = "main")

    repository_ctx.file("BUILD.bazel", executable = False)
    repository_ctx.file(
        "defs.bzl.tpl",
        content = ("RESIM_VERSION = \"{RESIM_VERSION}\"\n" +
                   "RESIM_BRANCH = \"{RESIM_BRANCH}\"\n"),
        executable = False,
    )
    repository_ctx.template(
        "defs.bzl",
        "defs.bzl.tpl",
        substitutions = {
            "{RESIM_BRANCH}": branch,
            "{RESIM_VERSION}": version,
        },
        executable = False,
    )

resim_version = repository_rule(
    environ = [
        "RESIM_VERSION",
        "RESIM_BRANCH",
    ],
    implementation = _resim_version_impl,
)

def _extension_impl(_):
    resim_version(name = "resim_version")

resim_version_extension = module_extension(
    implementation = _extension_impl,
)
