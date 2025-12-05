# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

py_console_script_binary(
    name = "pybind11-stubgen",
    pkg = "@resim_pkg_deps//pybind11_stubgen",
    visibility = ["//visibility:public"],
)

py_console_script_binary(
    name = "openapi-python-client",
    pkg = "@resim_python_client_dev_deps//openapi_python_client",
)
