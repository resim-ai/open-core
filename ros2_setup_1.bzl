# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Macro to help bring in ros2 transitive dependendencies at the third level.
"""

load(
    "@com_github_mvukov_rules_ros2//repositories:deps.bzl",
    "PIP_ANNOTATIONS",
)
load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_ros2_python//:defs.bzl", python_interpreter_target = "interpreter")

def ros2_setup_1():
    """Macro to help bring in ros2 transitive dependendencies at the third level.
    """
    pip_parse(
        name = "rules_ros2_pip_deps",
        annotations = PIP_ANNOTATIONS,
        python_interpreter_target = python_interpreter_target,
        requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
    )
