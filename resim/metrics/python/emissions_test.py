# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit tests for emissions.py.
"""

import json
import tempfile
import unittest
from pathlib import Path
from typing import Any

import numpy as np

from resim.metrics.python.emissions import emit
from resim.metrics.python.metrics_utils import Timestamp


class EmissionsTest(unittest.TestCase):
    """Test cases for the emissions module."""

    def setUp(self) -> None:
        """Set up test environment."""
        # trunk-ignore(pylint/R1732)
        self.temp_dir = tempfile.TemporaryDirectory()
        self.temp_path = Path(self.temp_dir.name) / "emissions.ndjson"

    def tearDown(self) -> None:
        """Clean up test environment."""
        self.temp_dir.cleanup()

    def test_emit_basic(self) -> None:
        """Test basic emission functionality."""
        topic_name = "test_topic"
        data = {"value": 42, "name": "test"}

        emit(topic_name, data, file_path=self.temp_path)

        # Verify file was created and contains the expected data
        self.assertTrue(self.temp_path.exists())

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertEqual(emission["$metadata"]["topic"], topic_name)
            self.assertEqual(emission["$data"], data)
            self.assertNotIn("timestamp", emission["$metadata"])

    def test_emit_with_timestamp(self) -> None:
        """Test emission with timestamp."""
        topic_name = "test_topic"
        data = {"value": 42}
        timestamp = 12345

        emit(topic_name, data, timestamp=timestamp, file_path=self.temp_path)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertEqual(emission["$metadata"]["topic"], topic_name)
            self.assertEqual(emission["$data"], data)
            self.assertEqual(emission["$metadata"]["timestamp"], timestamp)

    def test_emit_with_timestamp_object(self) -> None:
        """Test emission with Timestamp object."""
        topic_name = "test_topic"
        data = {"value": 42}
        timestamp = Timestamp(secs=12, nanos=345)
        expected_timestamp = 12 * 1_000_000_000 + 345

        emit(topic_name, data, timestamp=timestamp, file_path=self.temp_path)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertEqual(emission["$metadata"]["topic"], topic_name)
            self.assertEqual(emission["$data"], data)
            self.assertEqual(emission["$metadata"]["timestamp"], expected_timestamp)

    def test_emit_with_timestamps_array(self) -> None:
        """Test emission with timestamps array."""
        topic_name = "test_topic"
        data: dict[str, list[Any]] = {"values": [1, 2, 3], "labels": ["a", "b", "c"]}
        timestamps = [1000, 2000, 3000]

        emit(topic_name, data, timestamps=timestamps, file_path=self.temp_path)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)

            for i, line in enumerate(content):
                emission = json.loads(line)
                self.assertEqual(emission["$metadata"]["topic"], topic_name)
                self.assertEqual(emission["$data"]["values"], data["values"][i])
                self.assertEqual(emission["$data"]["labels"], data["labels"][i])
                self.assertEqual(emission["$metadata"]["timestamp"], timestamps[i])

    def test_emit_with_timestamp_objects_array(self) -> None:
        """Test emission with array of Timestamp objects."""
        topic_name = "test_topic"
        data = {"values": [1, 2, 3]}
        timestamps = [
            Timestamp(secs=1, nanos=0),
            Timestamp(secs=2, nanos=0),
            Timestamp(secs=3, nanos=0),
        ]
        expected_timestamps = [1 * 1_000_000_000, 2 * 1_000_000_000, 3 * 1_000_000_000]

        emit(topic_name, data, timestamps=timestamps, file_path=self.temp_path)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)

            for i, line in enumerate(content):
                emission = json.loads(line)
                self.assertEqual(emission["$metadata"]["topic"], topic_name)
                self.assertEqual(emission["$data"]["values"], data["values"][i])
                self.assertEqual(
                    emission["$metadata"]["timestamp"], expected_timestamps[i]
                )

    def test_emit_with_numpy_timestamps(self) -> None:
        """Test emission with numpy array of timestamps."""
        topic_name = "test_topic"
        data = {"values": [1, 2, 3]}
        # Convert numpy array to standard Python list to avoid JSON serialization issues
        timestamps = [int(x) for x in np.array([1000, 2000, 3000])]

        emit(topic_name, data, timestamps=timestamps, file_path=self.temp_path)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)

            for i, line in enumerate(content):
                emission = json.loads(line)
                self.assertEqual(emission["$metadata"]["topic"], topic_name)
                self.assertEqual(emission["$data"]["values"], data["values"][i])
                self.assertEqual(emission["$metadata"]["timestamp"], timestamps[i])

    def test_emit_with_file_object(self) -> None:
        """Test emission with file object instead of path."""
        topic_name = "test_topic"
        data = {"value": 42}

        with open(self.temp_path, "a", encoding="utf8") as f:
            emit(topic_name, data, file=f)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertEqual(emission["$metadata"]["topic"], topic_name)
            self.assertEqual(emission["$data"], data)

    def test_emit_validation_errors(self) -> None:
        """Test validation errors in emit function."""
        topic_name = "test_topic"
        data = {"values": [1, 2, 3]}

        # Test both timestamp and timestamps provided
        with self.assertRaises(RuntimeError) as context:
            emit(topic_name, data, timestamp=1000, timestamps=[1000, 2000, 3000])
        self.assertIn(
            "Only one of timestamp or timestamps can be set",
            str(context.exception.__cause__),
        )

        # Test timestamps with non-list data values
        with self.assertRaises(RuntimeError) as context:
            emit(topic_name, {"value": 42}, timestamps=[1000, 2000])
        self.assertIn("All data values must be lists", str(context.exception.__cause__))

        # Test timestamps with mismatched lengths
        with self.assertRaises(RuntimeError) as context:
            emit(topic_name, {"values": [1, 2, 3]}, timestamps=[1000, 2000])
        self.assertIn(
            "All series must be the same length as the timestamps",
            str(context.exception.__cause__),
        )

        # Test timestamps with multi-dimensional array
        with self.assertRaises(RuntimeError) as context:
            emit(topic_name, data, timestamps=np.array([[1000], [2000], [3000]]))
        self.assertIn("timestamps must be a 1D array", str(context.exception.__cause__))

    def test_emit_runtime_error(self) -> None:
        """Test that RuntimeError is raised for other exceptions."""
        topic_name = "test_topic"
        data = {"value": 42}

        # Test with invalid file path
        with self.assertRaises(RuntimeError):
            emit(topic_name, data, file_path=Path("/nonexistent/directory/file.ndjson"))

    def test_emit_append_to_existing_file(self) -> None:
        """Test appending emissions to an existing file."""
        topic_name = "test_topic"
        data1 = {"value": 1}
        data2 = {"value": 2}

        # First emission
        emit(topic_name, data1, file_path=self.temp_path)

        # Second emission
        emit(topic_name, data2, file_path=self.temp_path)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 2)

            emission1 = json.loads(content[0])
            emission2 = json.loads(content[1])

            self.assertEqual(emission1["$data"], data1)
            self.assertEqual(emission2["$data"], data2)


if __name__ == "__main__":
    unittest.main()
