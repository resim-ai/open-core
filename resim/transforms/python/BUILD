# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_test")

cc_binary(
    name = "se3_python.so",
    srcs = ["se3_python.cc"],
    linkshared = True,
    deps = [
        "//resim/transforms:se3",
        "@pybind",
    ],
)

py_test(
    name = "se3_python_test",
    srcs = ["se3_python_test.py"],
    data = [
        ":se3_python.so",
        requirement("numpy"),
    ],
)
