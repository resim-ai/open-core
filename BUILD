# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@platforms//host:constraints.bzl", "HOST_CONSTRAINTS")

exports_files(["requirements.txt"])

platform(
    name = "aarch64_linux",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:aarch64",
    ],
)

config_setting(
    name = "nocross",
    constraint_values = HOST_CONSTRAINTS,
    visibility = ["//visibility:public"],
)
