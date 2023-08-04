# Copied from https://bazel.build/tutorials/cc-toolchain-config

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_toolchain", "cc_toolchain_suite")
load("@rules_python//python:defs.bzl", "py_test")
load(":cc_toolchain_config.bzl", "cc_toolchain_config")

cc_toolchain_config(name = "k8_toolchain_config")

cc_toolchain_suite(
    name = "clang_suite",
    toolchains = {
        "k8": ":k8_toolchain",
    },
    visibility = ["//visibility:public"],
)

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
