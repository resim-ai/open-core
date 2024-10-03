# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# Copied from https://bazel.build/tutorials/cc-toolchain-config

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_toolchain")
load("@rules_python//python:defs.bzl", "py_test")
load(":cc_toolchain_config.bzl", "cc_toolchain_config", "cc_toolchain_config_amd64_aarch64_cross")

cc_toolchain_config(name = "k8_toolchain_config")

cc_toolchain_config(name = "aarch64_toolchain_config")

cc_toolchain_config_amd64_aarch64_cross(name = "amd64_aarch64_cross_toolchain_config")

filegroup(name = "empty")

cc_toolchain(
    name = "k8_toolchain",
    all_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    linker_files = ":empty",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    toolchain_config = ":k8_toolchain_config",
    toolchain_identifier = "k8-toolchain",
    visibility = ["//visibility:public"],
)

cc_toolchain(
    name = "aarch64_toolchain",
    all_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    linker_files = ":empty",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    toolchain_config = ":aarch64_toolchain_config",
    toolchain_identifier = "aarch64-toolchain",
    visibility = ["//visibility:public"],
)

cc_toolchain(
    name = "amd64_aarch64_cross_toolchain",
    all_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    linker_files = ":empty",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    toolchain_config = ":amd64_aarch64_cross_toolchain_config",
    toolchain_identifier = "amd64-aarch64-cross-toolchain",
    visibility = ["//visibility:public"],
)

toolchain(
    name = "cc_toolchain_for_k8",
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    toolchain = ":k8_toolchain",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)

toolchain(
    name = "cc_toolchain_for_aarch64",
    exec_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    toolchain = ":aarch64_toolchain",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)

toolchain(
    name = "aarch64_cross_toolchain_for_amd64",
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    toolchain = ":amd64_aarch64_cross_toolchain",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)

# Tests to assert that the expected toolchain is being used.
cc_library(
    name = "check_compiler",
    hdrs = ["check_compiler.hh"],
)

cc_test(
    name = "check_compiler_test",
    srcs = ["check_compiler_test.cc"],
    deps = [
        ":check_compiler",
        "@com_google_googletest//:gtest_main",
    ],
)

py_test(
    name = "check_python_test",
    srcs = ["check_python_test.py"],
)
