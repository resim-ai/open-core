# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import unittest
import os
import tempfile

import resim.ros2.resim_log_from_ros2_python as rfr2
import resim.ros2.resim_log_from_ros2_test_helpers_python as rfr2_test


class ResimLogFromRos2PythonTest(unittest.TestCase):
    def setUp(self) -> None:
        pass
        os.environ["AMENT_PREFIX_PATH"] = "resim/ros2/resim_log_from_ros2_python_test_ament/"

    def test_resim_log_from_ros2_python(self) -> None:
        with tempfile.TemporaryDirectory() as dir:
            in_path = os.path.join(dir, "in.mcap")
            out_path = os.path.join(dir, "out.mcap")            
            rfr2_test.populate_log(in_path)
            rfr2.resim_log_from_ros2('resim/ros2/default_converter_plugin.so',
                                     in_path,
                                     out_path)
            rfr2_test.verify_log_contents(out_path)

if __name__ == '__main__':
    unittest.main()
