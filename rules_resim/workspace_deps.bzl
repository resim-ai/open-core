# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Dependencies of rules_resim to be used in WORKSPACE files.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", _http_archive = "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load("@rules_resim//:extensions.bzl", "resim_cli")

def http_archive(**kwargs):
    maybe(_http_archive, **kwargs)

def rules_resim_repositories(cli_version = "v0.30.0"):
    """Bring in repositories that rules_resim() depends on.

    Args:
        cli_version: The version of the CLI to use. Defaults to v0.30.0.
    """

    resim_cli(name = "resim_cli", version = cli_version)

    http_archive(
        name = "platforms",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/platforms/releases/download/1.0.0/platforms-1.0.0.tar.gz",
            "https://github.com/bazelbuild/platforms/releases/download/1.0.0/platforms-1.0.0.tar.gz",
        ],
        sha256 = "3384eb1c30762704fbe38e440204e114154086c8fc8a8c2e3e28441028c019a8",
    )
