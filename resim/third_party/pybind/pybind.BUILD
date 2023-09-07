# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "pybind",
    hdrs = glob(["pybind11/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@python3_10//:python_headers",
    ],
)
