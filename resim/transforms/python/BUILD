# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")

pybind_extension(
    name = "quaternion",
    srcs = ["quaternion.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

py_test(
    name = "quaternion_test",
    srcs = ["quaternion_test.py"],
    data = [
        ":quaternion.so",
    ],
    deps = [
        requirement("numpy"),
    ],
)

pybind_extension(
    name = "se3_python",
    srcs = ["se3_python.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":so3_python.so",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "@libeigen//:eigen",
    ],
)

pybind_extension(
    name = "so3_python",
    srcs = ["so3_python.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":quaternion.so",
        "//resim/transforms:so3",
        "@libeigen//:eigen",
    ],
)

py_test(
    name = "se3_python_test",
    srcs = ["se3_python_test.py"],
    data = [
        ":se3_python.so",
        ":so3_python.so",
    ],
    deps = [
        requirement("numpy"),
    ],
)

py_test(
    name = "so3_python_test",
    srcs = ["so3_python_test.py"],
    data = [
        ":so3_python.so",
    ],
    deps = [
        requirement("numpy"),
    ],
)
