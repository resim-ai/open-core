# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Macro to help bring in transitive dependencies of this workspace.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load(
    "@com_github_mvukov_rules_ros2//repositories:deps.bzl",
    "ros2_deps",
)
load(
    "@com_github_mvukov_rules_ros2//repositories:repositories.bzl",
    "ros2_repositories",
)
load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")
load("@hedron_make_cc_https_easy//:transitive_workspace_setup.bzl", "hedron_keep_cc_https_easy")
load("@hedron_make_cc_https_easy//:workspace_setup.bzl", "hedron_make_cc_https_easy")
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")
load("@rules_fuzzing//fuzzing:repositories.bzl", "rules_fuzzing_dependencies")
load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")
load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")
load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_python//python:repositories.bzl", "python_register_toolchains")

def resim_core_transitive_dependencies():
    """Macro to help bring in transitive dependencies of this workspace.
    """
    bazel_skylib_workspace()

    rules_foreign_cc_dependencies()

    python_register_toolchains(
        name = "python3_10",
        # Available versions are listed in @rules_python//python:versions.bzl.
        # We recommend using the same version your team is already standardized on.
        python_version = "3.10",
        ignore_root_user_error = True,
        register_coverage_tool = True,
    )

    hedron_compile_commands_setup()

    rules_proto_dependencies()
    rules_proto_toolchains()

    rules_pkg_dependencies()

    rules_fuzzing_dependencies()

    hedron_make_cc_https_easy()

    # Requires the com_github_nelhage_rules_boost dep
    hedron_keep_cc_https_easy()

    ros2_repositories()

    ros2_deps()

    python_register_toolchains(
        name = "rules_ros2_python",
        python_version = "3.10",
        ignore_root_user_error = True,
    )

    # Create a central repo that knows about the dependencies needed from
    # requirements_lock.txt.
    pip_parse(
        name = "resim_python_deps",
        requirements_lock = "@resim_open_core//:requirements_lock.txt",
    )
