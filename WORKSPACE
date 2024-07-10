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

load("@resim-python-client//:deps_0.bzl", "resim_python_client_dependencies_0")

resim_python_client_dependencies_0()

load("@resim-python-client//:deps_1.bzl", "resim_python_client_dependencies_1")

resim_python_client_dependencies_1()

load("@resim-python-client//:deps_2.bzl", "resim_python_client_dependencies_2")

resim_python_client_dependencies_2()

load("@resim-python-client//:deps_3.bzl", "resim_python_client_dependencies_3")

resim_python_client_dependencies_3()

http_archive(
    name = "aspect_bazel_lib",
    sha256 = "f5ea76682b209cc0bd90d0f5a3b26d2f7a6a2885f0c5f615e72913f4805dbb0d",
    strip_prefix = "bazel-lib-2.5.0",
    url = "https://github.com/aspect-build/bazel-lib/releases/download/v2.5.0/bazel-lib-v2.5.0.tar.gz",
)

load("@aspect_bazel_lib//lib:repositories.bzl", "aspect_bazel_lib_dependencies", "aspect_bazel_lib_register_toolchains")

# Required bazel-lib dependencies

aspect_bazel_lib_dependencies()

# Register bazel-lib toolchains

aspect_bazel_lib_register_toolchains()

http_archive(
    name = "aspect_rules_py",
    sha256 = "e1d1023bc9ba8545dc87c6df10508d9d7c20f489f5e5c5c1e16380b33c013485",
    strip_prefix = "rules_py-0.5.0",
    url = "https://github.com/aspect-build/rules_py/releases/download/v0.5.0/rules_py-v0.5.0.tar.gz",
)

http_archive(
    name = "rules_oci",
    sha256 = "d41d0ba7855f029ad0e5ee35025f882cbe45b0d5d570842c52704f7a47ba8668",
    strip_prefix = "rules_oci-1.4.3",
    url = "https://github.com/bazel-contrib/rules_oci/releases/download/v1.4.3/rules_oci-v1.4.3.tar.gz",
)

load("@rules_oci//oci:dependencies.bzl", "rules_oci_dependencies")

rules_oci_dependencies()

load("@rules_oci//oci:repositories.bzl", "LATEST_CRANE_VERSION", "oci_register_toolchains")

oci_register_toolchains(
    name = "oci",
    crane_version = LATEST_CRANE_VERSION,
)

# You can pull your base images using oci_pull like this:
load("@rules_oci//oci:pull.bzl", "oci_pull")

oci_pull(
    name = "ubuntu",
    image = "ubuntu:jammy",
    platforms = [
        "linux/arm64/v8",
        "linux/amd64",
    ],
)
