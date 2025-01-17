# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")
load("//bazel:python.bzl", "pybind_extension")

pybind_extension(
    name = "polygon_distance_python",
    srcs = ["polygon_distance_python.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/geometry:polygon_distance",
    ],
)

py_test(
    name = "polygon_distance_test",
    srcs = ["polygon_distance_test.py"],
    data = [
        ":polygon_distance_python.so",
    ],
    deps = [
        requirement("numpy"),
    ],
)
