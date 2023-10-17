# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import unittest
import os

import resim.msg.resim_log_from_ros2_python as rfr2


class ResimLogFromRos2PythonTest(unittest.TestCase):
    def setUp(self) -> None:
        os.environ["AMENT_PREFIX_PATH"] = "resim/msg/resim_log_from_ros2_python_ament/"

    def test_resim_log_from_ros2_python(self) -> None:
        rfr2.resim_log_from_ros2('resim/ros2/default_converter_plugin.so', '/workspaces/log.mcap', 'out.mcap')

if __name__ == '__main__':
    unittest.main()
