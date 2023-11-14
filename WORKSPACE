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

# Golang protobuf dependencies
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "io_bazel_rules_go",
    sha256 = "51dc53293afe317d2696d4d6433a4c33feedb7748a9e352072e2ec3c0dafd2c6",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.40.1/rules_go-v0.40.1.zip",
        "https://github.com/bazelbuild/rules_go/releases/download/v0.40.1/rules_go-v0.40.1.zip",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains", "go_rules_dependencies")

go_rules_dependencies()

go_register_toolchains(version = "1.20.7")
