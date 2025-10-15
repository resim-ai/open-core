# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
# ReSim Extensions
"""

load("@rules_resim//private:api_spec.bzl", _resim_api_spec = "resim_api_spec")
load("@rules_resim//private:cli.bzl", _resim_cli = "resim_cli")

resim_cli = _resim_cli
resim_api_spec = _resim_api_spec

def _extension_impl(_):
    resim_cli(name = "resim_cli")
    resim_api_spec(name = "resim_api_spec")

resim_extension = module_extension(
    implementation = _extension_impl,
)
