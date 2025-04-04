# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# Copied from https://bazel.build/tutorials/cc-toolchain-config
# with updated clang version 19 paths
"""C++ toolchain config"""

load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")
load(
    "@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl",
    "feature",
    "flag_group",
    "flag_set",
    "tool_path",
)

all_link_actions = [
    ACTION_NAMES.cpp_link_executable,
    ACTION_NAMES.cpp_link_dynamic_library,
    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
]

all_cpp_actions = [
    ACTION_NAMES.cpp_compile,
    ACTION_NAMES.cc_flags_make_variable,
    ACTION_NAMES.cpp_module_codegen,
    ACTION_NAMES.cpp_header_parsing,
    ACTION_NAMES.cpp_module_compile,
]

all_target_actions = [
    ACTION_NAMES.cpp_compile,
    ACTION_NAMES.c_compile,
    ACTION_NAMES.cpp_link_executable,
    ACTION_NAMES.cpp_link_dynamic_library,
    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
]

def _toolchain_config_helper(
        ctx,
        cxx_builtin_include_directories = [],
        target_cpu = "k8",
        target_triple = ""):
    tool_paths = [
        tool_path(
            name = "gcc",
            path = "/usr/bin/clang-19",
        ),
        tool_path(
            name = "ld",
            path = "/usr/bin/ld",
        ),
        tool_path(
            name = "ar",
            path = "/usr/bin/ar",
        ),
        tool_path(
            name = "cpp",
            path = "/usr/bin/clang-cpp-19",
        ),
        tool_path(
            name = "gcov",
            path = "/usr/bin/llvm-profdata-19",
        ),
        tool_path(
            name = "llvm-cov",
            path = "/usr/bin/llvm-cov-19",
        ),
        tool_path(
            name = "llvm-profdata",
            path = "/usr/bin/llvm-profdata-19",
        ),
        tool_path(
            name = "nm",
            path = "/usr/bin/nm",
        ),
        tool_path(
            name = "objdump",
            path = "/usr/bin/objdump",
        ),
        tool_path(
            name = "strip",
            path = "/usr/bin/strip",
        ),
    ]
    features = [
        feature(
            name = "default_linker_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = ([
                        flag_group(
                            flags = [
                                "-lm",
                                "-lstdc++",
                            ],
                        ),
                    ]),
                ),
            ],
        ),
        feature(
            name = "default_cpp_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_cpp_actions,
                    flag_groups = ([
                        flag_group(
                            flags = [
                                "-std=c++20",
                                "-Wall",
                            ],
                        ),
                    ]),
                ),
            ],
        ),
        feature(
            name = "supports_pic",
            enabled = True,
        ),
        feature(
            name = "supports_dynamic_linker",
            enabled = True,
        ),
    ]
    if target_triple != "":
        features.append(
            feature(
                name = "aarch_cross",
                enabled = True,
                flag_sets = [
                    flag_set(
                        actions = all_target_actions,
                        flag_groups = ([
                            flag_group(
                                flags = [
                                    "--target={}".format(target_triple),
                                ],
                            ),
                        ]),
                    ),
                ],
            ),
        )
    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        cxx_builtin_include_directories = cxx_builtin_include_directories,
        features = features,
        toolchain_identifier = "local",
        host_system_name = "local",
        target_system_name = "local",
        target_cpu = target_cpu,
        target_libc = "unknown",
        compiler = "clang",
        abi_version = "unknown",
        abi_libc_version = "unknown",
        tool_paths = tool_paths,
    )

def _impl(ctx):
    return _toolchain_config_helper(
        ctx,
        [
            "/usr/lib/llvm-19/lib/clang/19/include",
            "/usr/include",
        ],
    )

def _aarch64_impl(ctx):
    return _toolchain_config_helper(
        ctx,
        [
            "/usr/lib/llvm-19/lib/clang/19/include",
            "/usr/include",
        ],
    )

def _amd64_aarch64_cross_impl(ctx):
    return _toolchain_config_helper(
        ctx,
        [
            "/usr/lib/llvm-19/lib/clang/19/include",
            "/usr/aarch64-linux-gnu/include",
        ],
        "aarch64",
        "aarch64-linux-gnu",
    )

cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)

cc_toolchain_config_aarch64 = rule(
    implementation = _aarch64_impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)

cc_toolchain_config_amd64_aarch64_cross = rule(
    implementation = _amd64_aarch64_cross_impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)
