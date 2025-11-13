# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "const",
    srcs = ["const.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk/auth:const",
    ],
)

py_library(
    name = "device_code_client",
    srcs = ["device_code_client.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk/auth:device_code_client",
    ],
)

py_library(
    name = "username_password_client",
    srcs = ["username_password_client.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk/auth:username_password_client",
    ],
)

py_library(
    name = "check_expiration",
    srcs = ["check_expiration.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk/auth:check_expiration",
    ],
)

py_test(
    name = "imports_test",
    srcs = ["imports_test.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":check_expiration",
        ":const",
        ":device_code_client",
        ":username_password_client",
    ],
)
