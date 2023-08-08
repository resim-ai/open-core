# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Macro to help bring in ros2 transitive dependendencies at the fourth level.
"""

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

def ros2_setup_2():
    """Macro to help bring in ros2 transitive dependendencies at the fourth level.
    """
    install_rules_ros2_pip_deps()
