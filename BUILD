# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@bazel_skylib//rules:common_settings.bzl", "bool_flag")
load("@platforms//host:constraints.bzl", "HOST_CONSTRAINTS")

exports_files(["requirements.txt"])

platform(
    name = "x86_64_linux",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
)

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

# Use the system uuid library optionally. Typically, we don't want to do this,
# but in certain cases (e.g. emscripten) it's convenient. In the emscripten
# case, there's a stub of it in the sysroot that we want to utilize.
bool_flag(
    name = "use_system_uuid",
    build_setting_default = False,
    visibility = ["//visibility:public"],
)
