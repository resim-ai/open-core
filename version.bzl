# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The version of open-core"""

def _resim_version_impl(repository_ctx):
    version = repository_ctx.os.environ.get("RESIM_VERSION", default = "0.0.0")
    if version.startswith("v"):
        version = version[1:]
    repository_ctx.file("BUILD.bazel", executable = False)
    repository_ctx.file(
        "defs.bzl.tpl",
        content = "RESIM_VERSION = \"{RESIM_VERSION}\"\n",
        executable = False,
    )
    repository_ctx.template(
        "defs.bzl",
        "defs.bzl.tpl",
        substitutions = {"{RESIM_VERSION}": version},
        executable = False,
    )

resim_version = repository_rule(
    environ = [
        "RESIM_VERSION",
    ],
    implementation = _resim_version_impl,
)

def _extension_impl(_):
    resim_version(name = "resim_version")

resim_version_extension = module_extension(
    implementation = _extension_impl,
)
