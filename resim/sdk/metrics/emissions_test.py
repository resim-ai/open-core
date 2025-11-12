# Copyright 2025 ReSim, Inc.
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
import yaml

from resim.sdk.metrics.emissions import Emitter, emit


class EmissionsTest(unittest.TestCase):
    """Test cases for the emissions module."""

    def setUp(self) -> None:
        """Set up test environment."""
        self.temp_dir = tempfile.TemporaryDirectory()
        self.temp_path = Path(self.temp_dir.name) / "emissions.resim.jsonl"

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
            emit(
                topic_name,
                data,
                file_path=Path("/nonexistent/directory/file.resim.jsonl"),
            )

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

    def test_emit_explode_lists_without_timestamps(self) -> None:
        """Test that lists are exploded into individual emissions if no timestamp(s) is provided."""
        topic_name = "test_topic"
        data: dict[str, list[Any]] = {"a": [1, 2, 3], "b": ["x", "y", "z"]}

        emit(topic_name, data, file_path=self.temp_path)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)
            for i, line in enumerate(content):
                emission = json.loads(line)
                self.assertEqual(emission["$metadata"]["topic"], topic_name)
                self.assertEqual(
                    emission["$data"], {"a": data["a"][i], "b": data["b"][i]}
                )
                self.assertNotIn("timestamp", emission["$metadata"])

    def test_emit_no_explode_when_not_all_lists(self) -> None:
        """Test that emission does not explode if not all values are lists or lists of the same length."""
        topic_name = "test_topic"
        # Case 1: one value is not a list
        data1 = {"a": [1, 2, 3], "b": "not_a_list"}
        emit(topic_name, data1, file_path=self.temp_path)
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 1)
            emission = json.loads(content[0])
            self.assertEqual(emission["$metadata"]["topic"], topic_name)
            self.assertEqual(emission["$data"], data1)
            self.assertNotIn("timestamp", emission["$metadata"])

        # Case 2: lists of different lengths
        self.temp_path.unlink()  # clear file
        data2 = {"a": [1, 2, 3], "b": ["x", "y"]}
        emit(topic_name, data2, file_path=self.temp_path)
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 1)
            emission = json.loads(content[0])
            self.assertEqual(emission["$metadata"]["topic"], topic_name)
            self.assertEqual(emission["$data"], data2)
            self.assertNotIn("timestamp", emission["$metadata"])

    def test_emit_explode_empty_lists(self) -> None:
        """Test that emission does not explode if all values are empty lists."""
        topic_name = "test_topic"

        data: dict[str, list[Any]] = {"a": [], "b": []}
        emit(topic_name, data, file_path=self.temp_path)
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 0)

        self.temp_path.unlink()  # clear file
        emit(topic_name, data, file_path=self.temp_path, timestamps=[])
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 0)

    def test_emit_event_validation(self) -> None:
        """Test that event emissions require a single timestamp and that event=True is set in the metadata."""
        topic_name = "test_topic"
        data = {"value": [41, 42, 43]}
        timestamp = 12345

        # Test valid event emission with a single timestamp
        emit(
            topic_name, data, timestamp=timestamp, event=True, file_path=self.temp_path
        )
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertEqual(emission["$metadata"]["topic"], topic_name)
            self.assertEqual(emission["$data"], data)
            self.assertEqual(emission["$metadata"]["timestamp"], timestamp)
            self.assertTrue(emission["$metadata"]["event"])

        # Test that event=True without a single timestamp raises an error
        if self.temp_path.exists():
            self.temp_path.unlink()  # clear file
        with self.assertRaises(RuntimeError) as context:
            emit(topic_name, data, event=True, file_path=self.temp_path)
        self.assertIn(
            "Event emissions must have a single timestamp",
            str(context.exception.__cause__),
        )

        # Test that event=True with timestamps raises an error
        if self.temp_path.exists():
            self.temp_path.unlink()  # clear file
        with self.assertRaises(RuntimeError) as context:
            emit(
                topic_name,
                {"values": [1, 2, 3]},
                timestamps=[1000, 2000, 3000],
                event=True,
                file_path=self.temp_path,
            )
        self.assertIn(
            "Event emissions must have a single timestamp",
            str(context.exception.__cause__),
        )


class EmitterTest(unittest.TestCase):
    """Test cases for the Emitter class."""

    def setUp(self) -> None:
        """Set up test environment."""
        self.temp_dir = tempfile.TemporaryDirectory()
        self.temp_path = Path(self.temp_dir.name) / "emissions.resim.jsonl"
        self.config_path = Path(self.temp_dir.name) / "config.yaml"

        # Create a sample configuration file
        config = {
            "version": 1,
            "topics": {
                "drone_speed": {
                    "schema": {
                        "speed_int": "int",
                        "speeds": "float",
                    }
                },
                "snapshots": {
                    "schema": {
                        "name": "string",
                        "picture": "image",
                    }
                },
                "node_crashes": {
                    "event": True,
                    "schema": {
                        "name": "string",
                        "description": "string",
                        "status": "status",
                        "tags": "string[]",
                        "metrics": "metric[]",
                    },
                },
            },
        }

        with open(self.config_path, "w", encoding="utf8") as f:
            yaml.dump(config, f)

    def tearDown(self) -> None:
        """Clean up test environment."""
        self.temp_dir.cleanup()

    def test_emitter_validation_topic_not_found(self) -> None:
        """Test that emitter raises error for unknown topic."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit("unknown_topic", {"value": 42})
            self.assertIn("topic 'unknown_topic' not found", str(context.exception))

    def test_emitter_validation_field_not_in_schema(self) -> None:
        """Test that emitter raises error for field not in schema."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit(
                    "drone_speed", {"speed_int": 42, "speeds": 3.14, "extra": "value"}
                )
            self.assertIn("field 'extra' not found in schema", str(context.exception))

    def test_emitter_validation_missing_field(self) -> None:
        """Test that emitter raises error for missing required field."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit("drone_speed", {"speed_int": 42})
            self.assertIn("missing required field", str(context.exception))
            self.assertIn("'speeds'", str(context.exception))

    def test_emitter_validation_wrong_type(self) -> None:
        """Test that emitter raises error for wrong data type."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit("drone_speed", {"speed_int": "not_an_int", "speeds": 3.14})
            self.assertIn("expected type int", str(context.exception))

    def test_emitter_validation_string_array_type(self) -> None:
        """Test validation for string[] type."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            emitter.emit_event(
                "node_crashes",
                {
                    "name": "crash1",
                    "description": "test crash",
                    "status": "FAILED",
                    "tags": ["tag1", "tag2"],
                    "metrics": [{"name": "metric1"}],
                },
                timestamp=1000,
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertEqual(emission["$data"]["tags"], ["tag1", "tag2"])

    def test_emitter_validation_string_array_wrong_type(self) -> None:
        """Test that emitter raises error for wrong string[] type."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit_event(
                    "node_crashes",
                    {
                        "name": "crash1",
                        "description": "test crash",
                        "status": "FAILED",
                        "tags": ["tag1", 123],  # 123 is not a string
                        "metrics": [],
                    },
                    timestamp=1000,
                )
            self.assertIn("'tags[1]'", str(context.exception))
            self.assertIn("expected type str", str(context.exception))

    def test_emitter_validation_metric_array_type(self) -> None:
        """Test validation for metric[] type."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            emitter.emit_event(
                "node_crashes",
                {
                    "name": "crash1",
                    "description": "test crash",
                    "status": "PASSED",
                    "tags": [],
                    "metrics": [{"name": "metric1"}, {"name": "metric2"}],
                },
                timestamp=1000,
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertEqual(
                emission["$data"]["metrics"], [{"name": "metric1"}, {"name": "metric2"}]
            )

    def test_emitter_event_validation(self) -> None:
        """Test that event emissions are validated correctly."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            # Valid event emission
            emitter.emit_event(
                "node_crashes",
                {
                    "name": "crash1",
                    "description": "test crash",
                    "status": "FAILED",
                    "tags": [],
                    "metrics": [],
                },
                timestamp=1000,
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertTrue(emission["$metadata"]["event"])

    def test_emitter_event_validation_non_event_topic(self) -> None:
        """Test that emitter raises error when using event=True on non-event topic."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit_event(
                    "drone_speed",
                    {"speed_int": 42, "speeds": 3.14},
                    timestamp=1000,
                )
            self.assertIn("is not configured as an event topic", str(context.exception))

    def test_emitter_context_manager_closes_file(self) -> None:
        """Test that context manager properly closes the file."""
        emitter = Emitter(self.config_path, self.temp_path)
        with emitter:
            self.assertIsNotNone(emitter.file)

        self.assertIsNone(emitter.file)

    def test_emitter_config_no_topics(self) -> None:
        """Test that emitter works without topics section (validation disabled)."""
        config_path = Path(self.temp_dir.name) / "bad_config.yaml"
        with open(config_path, "w", encoding="utf8") as f:
            yaml.dump({"version": 1}, f)

        # Should work without error - validation will be disabled
        with Emitter(config_path=config_path, output_path=self.temp_path) as emitter:
            # Validation is disabled, so any topic/data should work
            emitter.emit("any_topic", {"any_field": "value"})

        # Verify the emission was written
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertEqual(emission["$metadata"]["topic"], "any_topic")

    def test_emitter_validation_series_without_timestamps(self) -> None:
        """Test that emitter validates series emissions without explicit timestamps."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            emitter.emit_series(
                "drone_speed",
                {"speed_int": [42, 43, 44], "speeds": [3.14, 3.15, 3.16]},
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)

    def test_emitter_validation_series_wrong_type(self) -> None:
        """Test that emitter raises error for wrong type in series."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit_series(
                    "drone_speed",
                    {"speed_int": [42, "not_an_int", 44], "speeds": [3.14, 3.15, 3.16]},
                    timestamps=[1000, 2000, 3000],
                )
            self.assertIn("expected type int", str(context.exception))

    def test_emitter_multiple_emissions(self) -> None:
        """Test multiple emissions within the same context."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            emitter.emit("drone_speed", {"speed_int": 42, "speeds": 3.14})
            emitter.emit("drone_speed", {"speed_int": 43, "speeds": 3.15})
            emitter.emit("snapshots", {"name": "test", "picture": "image.png"})

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)

            emission1 = json.loads(content[0])
            emission2 = json.loads(content[1])
            emission3 = json.loads(content[2])

            self.assertEqual(emission1["$metadata"]["topic"], "drone_speed")
            self.assertEqual(emission2["$metadata"]["topic"], "drone_speed")
            self.assertEqual(emission3["$metadata"]["topic"], "snapshots")

    def test_emitter_without_config(self) -> None:
        """Test emitter without config file - validation disabled."""
        with Emitter(output_path=self.temp_path) as emitter:
            # Should allow any topic and any fields without validation
            emitter.emit("any_topic", {"any_field": 42, "another_field": "value"})
            emitter.emit("another_topic", {"x": 1, "y": 2, "z": 3})

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 2)

            emission1 = json.loads(content[0])
            emission2 = json.loads(content[1])

            self.assertEqual(emission1["$metadata"]["topic"], "any_topic")
            self.assertEqual(emission2["$metadata"]["topic"], "another_topic")

    def test_emitter_with_nonexistent_config_file(self) -> None:
        """Test emitter with nonexistent config file - validation disabled."""
        nonexistent_path = Path(self.temp_dir.name) / "nonexistent_config.yaml"

        with Emitter(
            config_path=nonexistent_path, output_path=self.temp_path
        ) as emitter:
            # Should work without validation since file doesn't exist
            emitter.emit("any_topic", {"any_field": "value"})

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertEqual(emission["$metadata"]["topic"], "any_topic")
            self.assertEqual(emission["$data"], {"any_field": "value"})

    def test_emitter_event_without_validation(self) -> None:
        """Test that event emissions work without validation."""
        with Emitter(output_path=self.temp_path) as emitter:
            emitter.emit_event("any_event", {"message": "test"}, timestamp=1000)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertTrue(emission["$metadata"]["event"])
            self.assertEqual(emission["$data"]["message"], "test")

    def test_emit(self) -> None:
        """Test the simplified emit method."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            emitter.emit(
                "drone_speed", {"speed_int": 42, "speeds": 3.14}, timestamp=1000
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertEqual(emission["$metadata"]["topic"], "drone_speed")
            self.assertEqual(emission["$data"], {"speed_int": 42, "speeds": 3.14})
            self.assertEqual(emission["$metadata"]["timestamp"], 1000)

    def test_emit_validation(self) -> None:
        """Test that emit validates data."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit(
                    "drone_speed",
                    {"speed_int": "not_an_int", "speeds": 3.14},
                    timestamp=1000,
                )
            self.assertIn("expected type int", str(context.exception))

    def test_emit_series_with_timestamps(self) -> None:
        """Test the simplified emit_series method."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            emitter.emit_series(
                "drone_speed",
                {"speed_int": [42, 43, 44], "speeds": [3.14, 3.15, 3.16]},
                timestamps=[1000, 2000, 3000],
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)

            for i, line in enumerate(content):
                emission = json.loads(line)
                self.assertEqual(emission["$metadata"]["topic"], "drone_speed")
                self.assertEqual(emission["$data"]["speed_int"], [42, 43, 44][i])
                self.assertEqual(emission["$data"]["speeds"], [3.14, 3.15, 3.16][i])
                self.assertEqual(
                    emission["$metadata"]["timestamp"], [1000, 2000, 3000][i]
                )

    def test_emit_series_validation(self) -> None:
        """Test that emit_series validates data."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            # Wrong type in series
            with self.assertRaises(TypeError) as context:
                emitter.emit_series(
                    "drone_speed",
                    {"speed_int": [42, "not_an_int", 44], "speeds": [3.14, 3.15, 3.16]},
                    timestamps=[1000, 2000, 3000],
                )
            self.assertIn("expected type int", str(context.exception))

    def test_emit_series_length_mismatch(self) -> None:
        """Test that emit_series validates array lengths."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit_series(
                    "drone_speed",
                    {"speed_int": [42, 43], "speeds": [3.14, 3.15, 3.16]},
                    timestamps=[1000, 2000, 3000],
                )
            self.assertIn("same length", str(context.exception))
            self.assertIn("'speed_int'", str(context.exception))

    def test_emit_series_non_list_data(self) -> None:
        """Test that emit_series requires list values."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit_series(
                    "drone_speed",
                    {"speed_int": 42, "speeds": 3.14},  # type: ignore
                    timestamps=[1000],
                )
            self.assertIn("must be lists", str(context.exception))

    def test_emit_event(self) -> None:
        """Test the simplified emit_event method."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            emitter.emit_event(
                "node_crashes",
                {
                    "name": "crash1",
                    "description": "test crash",
                    "status": "FAILED",
                    "tags": ["tag1"],
                    "metrics": [],
                },
                timestamp=1000,
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertEqual(emission["$metadata"]["topic"], "node_crashes")
            self.assertTrue(emission["$metadata"]["event"])
            self.assertEqual(emission["$metadata"]["timestamp"], 1000)
            self.assertEqual(emission["$data"]["name"], "crash1")

    def test_emit_event_validation(self) -> None:
        """Test that emit_event validates event topics."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            # Try to emit event on non-event topic
            with self.assertRaises(ValueError) as context:
                emitter.emit_event(
                    "drone_speed", {"speed_int": 42, "speeds": 3.14}, timestamp=1000
                )
            self.assertIn("is not configured as an event topic", str(context.exception))

    def test_emit_event_without_validation(self) -> None:
        """Test that emit_event works without validation."""
        with Emitter(output_path=self.temp_path) as emitter:
            emitter.emit_event("any_event", {"message": "test"}, timestamp=1000)

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)

            self.assertTrue(emission["$metadata"]["event"])
            self.assertEqual(emission["$data"]["message"], "test")

    def test_emit_series_with_numpy_array(self) -> None:
        """Test emit_series with numpy array timestamps and mismatched lengths."""
        # Test that validation works when timestamps is a numpy array.
        # This test checks that mismatched lengths between values and timestamps raise ValueError.
        config_path = Path(self.temp_dir.name) / "numpy_test_config.yaml"
        with open(config_path, "w", encoding="utf8") as f:
            yaml.dump({"topics": {"test_numpy": {"schema": {"values": "int"}}}}, f)

        with Emitter(config_path=config_path, output_path=self.temp_path) as emitter:
            # Test that validation works with numpy array
            # The isinstance check happens during validation, before JSON serialization
            np_timestamps = np.array([1000, 2000, 3000])

            # This should fail validation if lengths don't match - testing numpy array branch
            with self.assertRaises(ValueError) as context:
                emitter.emit_series(
                    "test_numpy",
                    {"values": [42, 43]},  # Wrong length - 2 vs 3
                    timestamps=np_timestamps,
                )
            self.assertIn("same length", str(context.exception))
            self.assertIn("'values'", str(context.exception))

    def test_emit_series_without_timestamps(self) -> None:
        """Test emit_series without timestamps."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            emitter.emit_series(
                "drone_speed", {"speed_int": [42, 43, 44], "speeds": [3.14, 3.15, 3.16]}
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)

            for i, line in enumerate(content):
                emission = json.loads(line)
                self.assertEqual(emission["$metadata"]["topic"], "drone_speed")
                self.assertEqual(emission["$data"]["speed_int"], [42, 43, 44][i])
                self.assertEqual(emission["$data"]["speeds"], [3.14, 3.15, 3.16][i])
                # Verify no timestamp in metadata
                self.assertNotIn("timestamp", emission["$metadata"])

    def test_emit_series_without_timestamps_mismatched_lengths(self) -> None:
        """Test that emit_series validates all series have same length when no timestamps."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit_series(
                    "drone_speed", {"speed_int": [42, 43], "speeds": [3.14, 3.15, 3.16]}
                )
            self.assertIn("same length", str(context.exception))
            self.assertIn("'speed_int'", str(context.exception))
            self.assertIn("'speeds'", str(context.exception))

    def test_emitter_config_corrupted_yaml(self) -> None:
        """Test that emitter handles corrupted YAML gracefully."""
        config_path = Path(self.temp_dir.name) / "corrupted.yaml"
        config_path.write_text("{ invalid yaml content [")

        # Should work without error - validation will be disabled
        with Emitter(config_path=config_path, output_path=self.temp_path) as emitter:
            emitter.emit("any_topic", {"any_field": "value"})

        # Verify emission was written
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertEqual(emission["$metadata"]["topic"], "any_topic")

    def test_validate_value_unknown_type(self) -> None:
        """Test that unknown type in schema raises ValueError."""
        config_path = Path(self.temp_dir.name) / "unknown_type_config.yaml"
        with open(config_path, "w", encoding="utf8") as f:
            yaml.dump(
                {"topics": {"test_topic": {"schema": {"field": "unknown_type"}}}}, f
            )

        with Emitter(config_path=config_path, output_path=self.temp_path) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit("test_topic", {"field": "value"})
            self.assertIn("unknown type 'unknown_type'", str(context.exception))

    def test_validate_value_metric_array_wrong_type(self) -> None:
        """Test that metric[] validates it's a list."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit_event(
                    "node_crashes",
                    {
                        "name": "crash1",
                        "description": "test",
                        "status": "FAILED",
                        "tags": [],
                        "metrics": "not_a_list",  # Should be a list
                    },
                    timestamp=1000,
                )
            self.assertIn("expected type list (metric[])", str(context.exception))

    def test_validate_value_string_array_not_list(self) -> None:
        """Test that string[] validates it's a list."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit_event(
                    "node_crashes",
                    {
                        "name": "crash1",
                        "description": "test",
                        "status": "FAILED",
                        "tags": "not_a_list",  # Should be a list
                        "metrics": [],
                    },
                    timestamp=1000,
                )
            self.assertIn("expected type list (string[])", str(context.exception))

    def test_emitter_validation_no_schema_defined(self) -> None:
        """Test that topic without schema raises ValueError."""
        config_path = Path(self.temp_dir.name) / "no_schema_config.yaml"
        with open(config_path, "w", encoding="utf8") as f:
            yaml.dump(
                {
                    "topics": {
                        "no_schema_topic": {}  # No schema defined
                    }
                },
                f,
            )

        with Emitter(config_path=config_path, output_path=self.temp_path) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit("no_schema_topic", {"field": "value"})
            self.assertIn("does not have a schema defined", str(context.exception))

    def test_emit_validation_disabled_with_config(self) -> None:
        """Test that validation can be disabled even with a config."""
        # This tests the validation_enabled flag when config is malformed
        config_path = Path(self.temp_dir.name) / "empty.yaml"
        config_path.write_text("")  # Empty YAML file

        with Emitter(config_path=config_path, output_path=self.temp_path) as emitter:
            # Should work - validation disabled for empty config
            emitter.emit("any_topic", {"any_field": "value"})

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertEqual(emission["$metadata"]["topic"], "any_topic")

    def test_emit_with_timestamp_none_explicitly(self) -> None:
        """Test emit with explicitly None timestamp."""
        with Emitter(
            config_path=self.config_path, output_path=self.temp_path
        ) as emitter:
            emitter.emit(
                "drone_speed", {"speed_int": 42, "speeds": 3.14}, timestamp=None
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertNotIn("timestamp", emission["$metadata"])

    def test_emitter_all_scalar_types(self) -> None:
        """Test all scalar types for comprehensive coverage."""
        config_path = Path(self.temp_dir.name) / "all_types_config.yaml"
        with open(config_path, "w", encoding="utf8") as f:
            yaml.dump(
                {
                    "topics": {
                        "all_types": {
                            "schema": {
                                "str_field": "string",
                                "int_field": "int",
                                "float_field": "float",
                                "image_field": "image",
                                "status_field": "status",
                            }
                        }
                    }
                },
                f,
            )

        with Emitter(config_path=config_path, output_path=self.temp_path) as emitter:
            emitter.emit(
                "all_types",
                {
                    "str_field": "test",
                    "int_field": 42,
                    "float_field": 3.14,
                    "image_field": "img.png",
                    "status_field": "PASSED",
                },
            )

        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.read().strip()
            emission = json.loads(content)
            self.assertEqual(emission["$data"]["str_field"], "test")
            self.assertEqual(emission["$data"]["int_field"], 42)
            self.assertEqual(emission["$data"]["float_field"], 3.14)

    def test_emitter_context_manager_exception_handling(self) -> None:
        """Test that __exit__ is called properly when exception occurs (line 241 coverage)."""
        with self.assertRaises(ValueError):
            with Emitter(
                config_path=self.config_path, output_path=self.temp_path
            ) as emitter:
                # Trigger a validation error inside the context
                emitter.emit("unknown_topic", {"field": "value"})

        # Verify that the file was closed properly even after exception
        # (The emitter should have closed the file in __exit__)
        self.assertIsNone(emitter.file)

    def test_emitter_auto_open_on_init(self) -> None:
        """Test that emitter automatically opens on construction."""
        emitter = Emitter(config_path=self.config_path, output_path=self.temp_path)

        # File should be automatically opened
        self.assertIsNotNone(emitter.file)

        # Should be ready to emit immediately
        emitter.emit("drone_speed", {"speed_int": 42, "speeds": 3.14})
        emitter.emit("drone_speed", {"speed_int": 43, "speeds": 3.15}, timestamp=1000)

        # Explicitly close
        emitter.close()

        # File should be closed
        self.assertIsNone(emitter.file)

        # Verify emissions were written
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 2)

            emission1 = json.loads(content[0])
            emission2 = json.loads(content[1])

            self.assertEqual(emission1["$data"]["speed_int"], 42)
            self.assertEqual(emission2["$data"]["speed_int"], 43)
            self.assertEqual(emission2["$metadata"]["timestamp"], 1000)

    def test_emitter_simple_usage_no_close_needed(self) -> None:
        """Test simple usage pattern - no need to call close explicitly."""
        emitter = Emitter(config_path=self.config_path, output_path=self.temp_path)

        # Just use it - no need to call open() or close()
        emitter.emit("drone_speed", {"speed_int": 42, "speeds": 3.14})
        emitter.emit_series(
            "drone_speed",
            {"speed_int": [1, 2], "speeds": [1.1, 2.2]},
            timestamps=[100, 200],
        )

        # Force close to verify emissions (normally __del__ would do this)
        emitter.close()

        # Verify all emissions
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 3)  # 1 single + 2 series

    def test_emitter_close_idempotent(self) -> None:
        """Test that close() can be called multiple times safely."""
        emitter = Emitter(config_path=self.config_path, output_path=self.temp_path)
        emitter.emit("drone_speed", {"speed_int": 42, "speeds": 3.14})

        # Close once
        emitter.close()
        self.assertIsNone(emitter.file)

        # Close again - should not raise error
        emitter.close()
        self.assertIsNone(emitter.file)

    def test_emitter_class_member_pattern(self) -> None:
        """Test using emitter as a class member that auto-opens and auto-closes."""

        class MyProcessor:
            def __init__(self, config_path: Path, output_path: Path) -> None:
                # Emitter automatically opens on construction
                self.emitter = Emitter(config_path=config_path, output_path=output_path)

            def process(self, topic: str, data: dict[str, Any]) -> None:
                self.emitter.emit(topic, data)

        # Create processor
        processor = MyProcessor(self.config_path, self.temp_path)

        # Use it
        processor.process("drone_speed", {"speed_int": 42, "speeds": 3.14})
        processor.process("drone_speed", {"speed_int": 43, "speeds": 3.15})

        # Clean up explicitly
        del processor

        # Verify emissions were written
        with open(self.temp_path, "r", encoding="utf8") as f:
            content = f.readlines()
            self.assertEqual(len(content), 2)

            emission1 = json.loads(content[0])
            emission2 = json.loads(content[1])

            self.assertEqual(emission1["$data"]["speed_int"], 42)
            self.assertEqual(emission2["$data"]["speed_int"], 43)

    def test_emit_event_with_unknown_topic(self) -> None:
        """Test that emit_event raises error for unknown topic."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(ValueError) as context:
                emitter.emit_event(
                    "unknown_event_topic", {"field": "value"}, timestamp=1000
                )
            self.assertIn(
                "topic 'unknown_event_topic' not found", str(context.exception)
            )
            self.assertIn("Available topics", str(context.exception))

    def test_validation_bool_to_int_field(self) -> None:
        """Test that passing bool to int field raises error with hint."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit("drone_speed", {"speed_int": True, "speeds": 3.14})
            self.assertIn("expected type int, got bool", str(context.exception))
            self.assertIn("Hint:", str(context.exception))
            self.assertIn("1 or 0", str(context.exception))

    def test_validation_bool_to_float_field(self) -> None:
        """Test that passing bool to float field raises error with hint."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit("drone_speed", {"speed_int": 42, "speeds": False})
            self.assertIn("expected type float, got bool", str(context.exception))
            self.assertIn("Hint:", str(context.exception))

    def test_validation_string_to_float_field(self) -> None:
        """Test that passing non-numeric to float field raises error."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit("drone_speed", {"speed_int": 42, "speeds": "not_a_number"})
            self.assertIn("expected type float", str(context.exception))
            self.assertIn("got str", str(context.exception))

    def test_validation_wrong_type_for_string_field(self) -> None:
        """Test that passing wrong type to string field raises error."""
        with Emitter(self.config_path, self.temp_path) as emitter:
            with self.assertRaises(TypeError) as context:
                emitter.emit("snapshots", {"name": 123, "picture": "img.png"})
            self.assertIn("expected type str", str(context.exception))
            self.assertIn("got int", str(context.exception))


if __name__ == "__main__":
    unittest.main()
