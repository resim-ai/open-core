# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Extension for the resim CLI."""

load("@rules_resim//private:cli.bzl", _resim_cli = "resim_cli")

resim_cli = _resim_cli

def _extension_impl(_):
    resim_cli(name = "resim_cli")

resim_cli_extension = module_extension(
    implementation = _extension_impl,
)
