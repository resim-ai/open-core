# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "const",
    srcs = ["const.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "device_code_client",
    srcs = ["device_code_client.py"],
    visibility = ["//visibility:public"],
    deps = [
        requirement("httpx"),
        ":check_expiration",
        ":const",
        "//resim/sdk:client",
    ],
)

py_test(
    name = "device_code_client_test",
    srcs = ["device_code_client_test.py"],
    deps = [
        ":device_code_client",
    ],
)

py_library(
    name = "username_password_client",
    srcs = ["username_password_client.py"],
    visibility = ["//visibility:public"],
    deps = [
        requirement("httpx"),
        ":check_expiration",
        ":const",
        "//resim/sdk:client",
    ],
)

py_test(
    name = "username_password_client_test",
    srcs = ["username_password_client_test.py"],
    deps = [
        ":username_password_client",
    ],
)

py_library(
    name = "check_expiration",
    srcs = ["check_expiration.py"],
    visibility = ["//visibility:public"],
    deps = [],
)

py_test(
    name = "check_expiration_test",
    srcs = ["check_expiration_test.py"],
    deps = [
        ":check_expiration",
    ],
)

py_library(
    name = "auth",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":device_code_client",
        ":username_password_client",
    ],
)
