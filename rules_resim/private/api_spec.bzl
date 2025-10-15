# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
ReSim OpenAPI Specification Fetching Rule
"""

def _resim_api_spec_impl(rctx):
    rctx.download(
        url = "https://api.resim.ai/v1/openapi.yaml",
        output = "openapi.yaml",
        executable = True,
    )
    rctx.file("BUILD", rctx.read(rctx.attr._build_file))

resim_api_spec = repository_rule(
    implementation = _resim_api_spec_impl,
    attrs = {
        "_build_file": attr.label(
            allow_single_file = True,
            default = ":api_spec.BUILD",
            doc = "BUILD file for the repo",
        ),
    },
)
