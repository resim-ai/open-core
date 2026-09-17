# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Repository rule exposing the versions passed in by the release workflows.

RESIM_VERSION is the open-core version: every resim_* wheel and the Go
bindings are stamped with it by release.yml. SIGNALFLAG_VERSION is the
SignalFlag Python SDK's own version, set by release-sdk.yml; the SDK is
released on its own cadence and never inherits the open-core version. Both
default to 0.0.0 for local builds.
"""

def _strip_v(version):
    return version[1:] if version.startswith("v") else version

def _resim_version_impl(repository_ctx):
    version = _strip_v(repository_ctx.os.environ.get("RESIM_VERSION", default = "0.0.0"))
    signalflag_version = _strip_v(repository_ctx.os.environ.get("SIGNALFLAG_VERSION", default = "0.0.0"))
    branch = repository_ctx.os.environ.get("RESIM_BRANCH", default = "main")

    repository_ctx.file("BUILD.bazel", executable = False)
    repository_ctx.file(
        "defs.bzl.tpl",
        content = ("RESIM_VERSION = \"{RESIM_VERSION}\"\n" +
                   "RESIM_BRANCH = \"{RESIM_BRANCH}\"\n" +
                   "SIGNALFLAG_VERSION = \"{SIGNALFLAG_VERSION}\"\n"),
        executable = False,
    )
    repository_ctx.template(
        "defs.bzl",
        "defs.bzl.tpl",
        substitutions = {
            "{RESIM_BRANCH}": branch,
            "{RESIM_VERSION}": version,
            "{SIGNALFLAG_VERSION}": signalflag_version,
        },
        executable = False,
    )

resim_version = repository_rule(
    environ = [
        "RESIM_VERSION",
        "RESIM_BRANCH",
        "SIGNALFLAG_VERSION",
    ],
    implementation = _resim_version_impl,
)

def _extension_impl(_):
    resim_version(name = "resim_version")

resim_version_extension = module_extension(
    implementation = _extension_impl,
)
