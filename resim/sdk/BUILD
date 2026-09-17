# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_python//python:defs.bzl", "py_library")

# Backwards-compatibility alias: `resim.sdk` -> `signalflag.sdk`.
py_library(
    name = "compat",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim:alias",
        "//signalflag/sdk",
    ],
)
