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

load("@rules_python//python:pip.bzl", "pip_parse")

# Create a central repo that knows about the dependencies needed from
# requirements_lock.txt.
pip_parse(
    name = "python_deps",
    requirements_lock = "//:requirements_lock.txt",
)

load("@python_deps//:requirements.bzl", "install_deps")

install_deps()
