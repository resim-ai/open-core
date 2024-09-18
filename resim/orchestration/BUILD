# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "resim_python_client_mocks",
    testonly = True,
    srcs = ["resim_python_client_mocks.py"],
    deps = [
        "@resim-python-client//:resim_python_client",
    ],
)

py_test(
    name = "resim_python_client_mocks_test",
    srcs = ["resim_python_client_mocks_test.py"],
    deps = [":resim_python_client_mocks"],
)

py_library(
    name = "kickoff_report",
    srcs = ["kickoff_report.py"],
    visibility = ["//pkg:__subpackages__"],
    deps = [
        "//resim/msg:detection_proto_py",
        "@resim-python-client//:resim_python_client",
    ],
)

py_test(
    name = "kickoff_report_test",
    srcs = ["kickoff_report_test.py"],
    deps = [
        ":kickoff_report",
        ":resim_python_client_mocks",
    ],
)
