# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit test for pybinding of resim_log_from_ros2
"""

import os
import tempfile
import unittest

import resim.ros2.resim_log_from_ros2_python as rfr2
import resim.ros2.resim_log_from_ros2_test_helpers_python as rfr2_test


class ResimLogFromRos2PythonTest(unittest.TestCase):
    """
    Unit test for pybinding of resim_log_from_ros2
    """

    def test_resim_log_from_ros2_python(self) -> None:
        """Test that we can correctly convert logs to the ReSim format."""
        with tempfile.TemporaryDirectory() as testdir:
            in_path = os.path.join(testdir, "in.mcap")
            out_path = os.path.join(testdir, "out.mcap")
            rfr2_test.populate_log(in_path)
            rfr2.resim_log_from_ros2(
                "resim/ros2/testing/resim_log_from_ros2_test_converter_plugin.so",
                in_path,
                out_path,
            )
            rfr2_test.verify_log_contents(out_path)


if __name__ == "__main__":
    unittest.main()
