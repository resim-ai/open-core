# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "device_code_client",
    srcs = ["device_code_client.py"],
    visibility = ["//visibility:public"],
    deps = [
        requirement("polling2"),
        requirement("requests"),
    ],
)

py_test(
    name = "device_code_client_test",
    srcs = ["device_code_client_test.py"],
    deps = [
        ":device_code_client",
    ],
)
