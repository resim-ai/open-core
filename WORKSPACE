# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

workspace(name = "resim_open_core")

# Due to the documented shortcomings of the WORKSPACE system (see
# https://bazel.build/external/overview), we are forced to call four functions
# in order to set up the workspace. This is because starlark macros cannot load
# other .bzl files. In other words, we cannot load bzl files from dependency
# X without first bringing X in with e.g. an http_archive call in a macro, and
# we cannot bring in X's dependencies without loading bzl files from it. This
# limitation compounds at each level of transitive dependency.

# In order to depend on this repository, you can simply copy/paste the rest of
# this file into your WORKSPACE files.
load("@resim_open_core//:deps.bzl", "resim_core_dependencies")

resim_core_dependencies()

load("@resim_open_core//:transitive_deps.bzl", "resim_core_transitive_dependencies")

resim_core_transitive_dependencies()

load("@resim_open_core//:ros2_setup_1.bzl", "ros2_setup_1")

ros2_setup_1()

load("@resim_open_core//:ros2_setup_2.bzl", "ros2_setup_2")

ros2_setup_2()

bind(
    name = "python_headers",
    actual = "@python3_10//:python_headers",
)

load("@resim_python_deps//:requirements.bzl", "install_deps")

install_deps()

load("@pybind11_bazel//:python_configure.bzl", "python_configure")
load("@python3_10//:defs.bzl", "interpreter")

python_configure(
    name = "local_config_python",
    python_interpreter_target = interpreter,
)

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "toolchains_llvm",
    canonical_id = "0.10.3",
    patch_args = ["-p1"],
    patches = ["@resim_open_core//resim/third_party/toolchains_llvm:abspath.patch"],
    sha256 = "b7cd301ef7b0ece28d20d3e778697a5e3b81828393150bed04838c0c52963a01",
    strip_prefix = "toolchains_llvm-0.10.3",
    url = "https://github.com/grailbio/bazel-toolchain/releases/download/0.10.3/toolchains_llvm-0.10.3.tar.gz",
)

load("@toolchains_llvm//toolchain:deps.bzl", "bazel_toolchain_dependencies")

bazel_toolchain_dependencies()

load("@toolchains_llvm//toolchain:rules.bzl", "llvm_toolchain")

llvm_toolchain(
    name = "llvm_toolchain",
    absolute_paths = True,
    cxx_standard = {"": "c++20"},
    link_libs = {"": ["-luuid"]},
    llvm_version = "14.0.0",
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()
