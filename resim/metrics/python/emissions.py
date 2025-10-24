# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import json
from io import TextIOWrapper
from pathlib import Path
from types import TracebackType
from typing import Any, Optional, Type, Union
from contextlib import AbstractContextManager

import numpy as np
import numpy.typing as npt
import yaml
import logging
from resim.metrics.python.metrics_utils import Timestamp

logger = logging.getLogger(__name__)

# Default output path for emissions
DEFAULT_EMISSIONS_PATH = Path("/tmp/resim/outputs/emissions.resim.jsonl")

# Export public API
__all__ = [
    "emit",
    "Emitter",
    "ReSimValidationError",
]


# Custom exception class for better error handling
class ReSimValidationError(ValueError, TypeError):
    """Exception for ReSim metrics validation errors.

    This exception inherits from both ValueError and TypeError to maintain backward
    compatibility with existing error handling code.

    This exception provides structured error information including:
    - Topic name
    - Field name (if applicable)
    - Series index (for series emissions)
    - Expected vs actual values
    - Helpful hints and suggestions

    Attributes:
        topic: The topic name where the error occurred
        field: The field name where the error occurred (if applicable)
        series_index: The index in a series where the error occurred (if applicable)
        hint: A helpful hint for fixing the error (if available)
    """

    def __init__(
        self,
        message: str,
        topic: Optional[str] = None,
        field: Optional[str] = None,
        series_index: Optional[int] = None,
        hint: Optional[str] = None,
    ):
        self.topic = topic
        self.field = field
        self.series_index = series_index
        self.hint = hint

        # Build a structured error message
        parts = []
        if series_index is not None:
            parts.append(f"Series index {series_index}")
        if topic:
            parts.append(f"topic '{topic}'")
        if field:
            parts.append(f"field '{field}'")

        if parts:
            full_message = f"Validation error ({', '.join(parts)}): {message}"
        else:
            full_message = f"Validation error: {message}"

        if hint:
            full_message += f"\n  Hint: {hint}"

        super().__init__(full_message)


def emit(
    topic_name: str,
    data: dict[str, Any],
    *,
    timestamp: Optional[Union[int, Timestamp]] = None,
    timestamps: Optional[
        Union[list[int], list[Timestamp], npt.NDArray[np.int_]]
    ] = None,
    event: bool = False,
    file_path: Path = DEFAULT_EMISSIONS_PATH,
    file: Optional[TextIOWrapper] = None,
) -> None:
    """
    Emit a single point or a series of datapoints to a file.

    Args:
        topic_name: The name of the topic to emit the data to.
        data: A dictionary of data to emit. If using a series of timestamps, all data values must be lists.
    Optional Args:
        timestamp: The timestamp of the data point to emit. Mutually exclusive with timestamps.
        timestamps: A list of timestamps to emit the data at. Mutually exclusive with timestamp.
        event: Annotates the emission as an event. Must be used with a single timestamp.
        file_path: The path to the file to emit the data to. Mutually exclusive with file.
        file: An optional file object to emit the data to. Mutually exclusive with file_path.
    """
    try:
        # only allow one of timestamp or timestamps to be set
        if timestamp is not None and timestamps is not None:
            raise ValueError("Only one of timestamp or timestamps can be set")

        # If event is True, ensure this is a single timestamp emission
        if event and (timestamp is None or timestamps is not None):
            raise ValueError(
                "Event emissions must have a single timestamp (no recursion or multiple timestamps)"
            )

        # If no timestamp(s) and all data values are lists of the same length, recursively emit each point
        if (
            timestamp is None
            and timestamps is None
            and not event
            and len(data) > 0
            and all(isinstance(v, list) for v in data.values())
        ):
            lengths = set(len(v) for v in data.values())
            if len(lengths) == 1:
                open_file = file
                if open_file is None:
                    open_file = open(file_path, "a", encoding="utf8")
                for i in range(lengths.pop()):  # pragma: no cover
                    scalar_data = {k: v[i] for k, v in data.items()}
                    emit(
                        topic_name,
                        scalar_data,
                        file=open_file,
                    )
                if file is None:
                    open_file.close()
                return  # pragma: no cover

        if timestamp is None and timestamps is not None:
            # assert all data values are mapped to lists of the same length
            if not all(isinstance(v, list) for v in data.values()):
                raise ValueError("All data values must be lists")

            if not all(len(timestamps) == len(series) for series in data.values()):
                raise ValueError("All series must be the same length as the timestamps")

            if isinstance(timestamps, np.ndarray) and timestamps.ndim != 1:
                raise ValueError("timestamps must be a 1D array")

            open_file = file
            if open_file is None:
                open_file = open(file_path, "a", encoding="utf8")

            for i, ts in enumerate(timestamps):  # pragma: no cover
                scalar_data = {k: v[i] for k, v in data.items()}
                emit(
                    topic_name,
                    scalar_data,
                    timestamp=ts,
                    file=open_file,
                )
            if file is None:
                open_file.close()
            return  # pragma: no cover

        # build the single point emission dictionary
        emission = {
            "$metadata": {
                "topic": topic_name,
            },
            "$data": data,
        }
        if timestamp is not None:
            if isinstance(timestamp, Timestamp):
                emission["$metadata"]["timestamp"] = timestamp.to_nanos()
            else:
                emission["$metadata"]["timestamp"] = timestamp
        # Set event flag if event is True and this is a single timestamp emission
        if event and timestamp is not None:
            emission["$metadata"]["event"] = True

        # write the emission to the output file
        open_file = file
        if open_file is None:
            open_file = open(file_path, "a", encoding="utf8")
        open_file.write(json.dumps(emission) + "\n")
        if file is None:
            open_file.close()

    except Exception as e:
        raise RuntimeError(f"Error emitting topic {topic_name}") from e


class Emitter(AbstractContextManager):
    """Emitter for metrics with optional validation against a YAML configuration.

    The Emitter automatically opens its output file on construction and closes it when
    destroyed or used as a context manager. If a config file exists, emissions are
    validated against the schema. If not, validation is disabled.

    Methods:
        emit(): Emit a single datapoint with optional timestamp
        emit_series(): Emit multiple datapoints with optional timestamps
        emit_event(): Emit an event with a required timestamp

    Examples:
        Simple usage (auto-open, auto-close)::

            emitter = Emitter(config_path="config.yaml")
            emitter.emit("drone_speed", {"speed_int": 42, "speeds": 3.14})
            # File automatically closed when emitter goes out of scope

        Context manager::

            with Emitter(config_path="config.yaml") as emitter:
                emitter.emit("drone_speed", {"speed_int": 42, "speeds": 3.14})
                emitter.emit_series(
                    "drone_speed",
                    {"speed_int": [42, 43, 44]},
                    timestamps=[1000, 2000, 3000]
                )
                emitter.emit_event("crash", {"severity": "high"}, timestamp=5000)

        As a class member::

            class MyProcessor:
                def __init__(self):
                    self.emitter = Emitter(config_path="config.yaml")

                def process(self, data):
                    self.emitter.emit("processed", data)
    """

    # Type mapping from schema types to Python types
    TYPE_MAPPING = {
        "string": str,
        "int": int,
        "float": float,
        "image": str,  # image references are stored as strings
        "status": str,  # status values are stored as strings
        "string[]": list,
        "metric[]": list,
    }

    def __init__(
        self,
        config_path: Optional[Union[str, Path]] = None,
        output_path: Union[str, Path] = DEFAULT_EMISSIONS_PATH,
    ):
        """
        Initialize and open the Emitter with optional configuration and output file path.

        The emitter is immediately ready to use after construction. The file will be
        automatically closed when the object is garbage collected or when close() is called.

        Args:
            config_path: Optional path to the YAML metrics configuration file. If the file doesn't exist, validation is disabled.
            output_path: Path to the output file for emissions. Defaults to /tmp/resim/outputs/emissions.resim.jsonl.
        """
        self.config_path = Path(config_path) if config_path is not None else None
        self.output_path = Path(output_path)
        self.topics: dict[str, dict[str, Any]] = {}
        self.file: Optional[TextIOWrapper] = None
        self.validation_enabled = False

        # Automatically open the emitter
        self._initialize()

    def _initialize(self) -> None:
        """
        Internal method to initialize the emitter: load configuration and open the output file.

        Called automatically by __init__.
        """
        # Try to load configuration if config_path is provided
        if self.config_path is not None:
            if self.config_path.is_file():
                logger.info(f"Loading config file {self.config_path} for validation.")
                try:
                    # Load and parse the YAML configuration file
                    with open(self.config_path, "r", encoding="utf8") as config_file:
                        config = yaml.safe_load(config_file)

                    # Extract topics from the configuration
                    # Note: yaml.safe_load() returns None for empty files
                    if config is not None and "topics" in config:
                        self.topics = config["topics"]
                        self.validation_enabled = True
                        logger.info(
                            f"Validation enabled with {len(self.topics)} topic(s) from config."
                        )
                    else:
                        logger.warning(
                            f"Config file {self.config_path} has no 'topics' key. Validation disabled."
                        )
                        self.validation_enabled = False
                except (yaml.YAMLError, IOError, OSError) as e:
                    # If loading fails, continue without validation but log the error
                    logger.warning(
                        f"Failed to load config file {self.config_path}: {e}. Validation disabled."
                    )
                    self.validation_enabled = False
            else:
                # Config file path provided but doesn't exist - disable validation
                logger.warning(
                    f"Config file {self.config_path} not found. Validation disabled."
                )
                self.validation_enabled = False
        else:
            # No config file provided - validation intentionally disabled
            logger.debug("No config file provided. Validation disabled.")
            self.validation_enabled = False

        # Open the output file
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = open(self.output_path, "a", encoding="utf8")

    def close(self) -> None:
        """
        Close the emitter: close the output file.

        This method is optional - the file will be automatically closed when the
        emitter is garbage collected. However, you can call it explicitly for
        immediate cleanup or when using a 'with' statement.

        It is safe to call multiple times (idempotent).

        Example:
            emitter = Emitter(config_path="config.yaml")
            emitter.emit("topic", {"field": "value"})
            emitter.close()  # Optional - will be called automatically anyway
        """
        if self.file is not None:
            self.file.close()
            self.file = None

    def __del__(self) -> None:
        """
        Destructor: ensure the file is closed when the object is garbage collected.
        """
        self.close()

    def __enter__(self) -> "Emitter":
        """
        Enter the context manager.

        Since the emitter is already opened in __init__, this just returns self.
        """
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        """
        Exit the context manager: close the file.
        """
        self.close()

    def _validate_event_topic(self, topic_name: str) -> None:
        """
        Validate that a topic is configured as an event topic.

        Args:
            topic_name: The name of the topic to check.

        Raises:
            ReSimValidationError: If validation is enabled and the topic is not found or not configured as an event.
        """
        if not self.validation_enabled:
            return

        if topic_name not in self.topics:
            available_topics = ", ".join(f"'{t}'" for t in self.topics.keys())
            raise ReSimValidationError(
                f"topic '{topic_name}' not found in configuration for emit_event. Available topics: {available_topics}",
                topic=topic_name,
            )

        topic_config = self.topics[topic_name]
        if not topic_config.get("event", False):
            raise ReSimValidationError(
                f"topic '{topic_name}' is not configured as an event topic",
                topic=topic_name,
                hint="Use emit() or emit_series() instead, or add 'event: true' to the topic configuration.",
            )

    def _validate_series_data(
        self,
        topic_name: str,
        data: dict[str, list[Any]],
        timestamps: Optional[Union[list[int], list[Timestamp], npt.NDArray[np.int_]]],
    ) -> None:
        """
        Validate a series of datapoints.

        Args:
            topic_name: The name of the topic.
            data: A dictionary where each value is a list of data points.
            timestamps: Optional list of timestamps corresponding to each data point.

        Raises:
            ReSimValidationError: If validation fails or array lengths don't match.
        """
        # Validate that all data values are lists
        non_list_fields = [k for k, v in data.items() if not isinstance(v, list)]
        if non_list_fields:
            non_list_str = ", ".join(f"'{f}'" for f in non_list_fields)
            raise ReSimValidationError(
                f"all data values must be lists in emit_series, but these fields are not: {non_list_str}",
                topic=topic_name,
            )

        # Determine the number of points
        if timestamps is not None:
            # Validate lengths match when timestamps are provided
            num_points = len(timestamps)

            mismatched = [(k, len(v)) for k, v in data.items() if len(v) != num_points]
            if mismatched:
                length_info = ", ".join(f"'{k}': {length}" for k, length in mismatched)
                raise ReSimValidationError(
                    f"all series must be the same length as timestamps (length {num_points}), "
                    f"but these fields have different lengths: {length_info}",
                    topic=topic_name,
                )
        else:
            # No timestamps provided - all series should be the same length
            lengths = {k: len(v) for k, v in data.items()}
            unique_lengths = set(lengths.values())
            if len(unique_lengths) != 1:
                length_info = ", ".join(
                    f"'{k}': {length}" for k, length in lengths.items()
                )
                raise ReSimValidationError(
                    f"all series must be the same length in emit_series, but got different lengths: {length_info}",
                    topic=topic_name,
                )
            num_points = unique_lengths.pop()

        # Validate each point in the series
        for i in range(num_points):
            point_data = {k: v[i] for k, v in data.items()}
            try:
                self._validate_data(topic_name, point_data)
            except ReSimValidationError as e:
                # Re-raise with series index context
                raise ReSimValidationError(
                    str(e),
                    topic=topic_name,
                    series_index=i,
                ) from e

    def _validate_value(
        self, value: Any, expected_type_str: str, field_name: str, topic_name: str
    ) -> None:
        """
        Validate a single value against an expected type.

        Args:
            value: The value to validate.
            expected_type_str: The expected type as a string (e.g., "int", "string[]").
            field_name: The name of the field being validated (for error messages).
            topic_name: The name of the topic (for error messages).

        Raises:
            ReSimValidationError: If the value does not match the expected type.
        """
        if expected_type_str not in self.TYPE_MAPPING:
            raise ReSimValidationError(
                f"config has unknown type '{expected_type_str}'",
                topic=topic_name,
                field=field_name,
            )

        # Helper to format value for display (limit size for readability)
        def format_value(v: Any) -> str:
            v_str = repr(v)
            return v_str if len(v_str) <= 50 else f"{v_str[:47]}..."

        # For list types, validate that the value is a list and check element types
        if expected_type_str == "string[]":
            if not isinstance(value, list):
                raise ReSimValidationError(
                    f"expected type list (string[]), got {type(value).__name__} with value {format_value(value)}",
                    topic=topic_name,
                    field=field_name,
                )
            for i, item in enumerate(value):
                if not isinstance(item, str):
                    raise ReSimValidationError(
                        f"expected type str, got {type(item).__name__} with value {format_value(item)}",
                        topic=topic_name,
                        field=f"{field_name}[{i}]",
                    )
        elif expected_type_str == "metric[]":
            if not isinstance(value, list):
                raise ReSimValidationError(
                    f"expected type list (metric[]), got {type(value).__name__} with value {format_value(value)}",
                    topic=topic_name,
                    field=field_name,
                )
            # For metric[], we just validate it's a list; the contents are flexible
        elif expected_type_str == "int":
            # Special handling for int: reject Booleans even though bool is a subclass of int
            if isinstance(value, bool):
                raise ReSimValidationError(
                    f"expected type int, got bool with value {value}",
                    topic=topic_name,
                    field=field_name,
                    hint="Use 1 or 0 instead of True or False for int fields.",
                )
            elif not isinstance(value, int):
                raise ReSimValidationError(
                    f"expected type int, got {type(value).__name__} with value {format_value(value)}",
                    topic=topic_name,
                    field=field_name,
                )
        elif expected_type_str == "float":
            if isinstance(value, bool):
                raise ReSimValidationError(
                    f"expected type float, got bool with value {value}",
                    topic=topic_name,
                    field=field_name,
                    hint="Use numeric values instead of True or False.",
                )
            # Allow both int and float for float fields (numeric flexibility)
            elif not isinstance(value, (int, float)):
                raise ReSimValidationError(
                    f"expected type float, got {type(value).__name__} with value {format_value(value)}",
                    topic=topic_name,
                    field=field_name,
                )
        else:
            # For other scalar types (string, image, status), validate the type directly
            expected_type = self.TYPE_MAPPING[expected_type_str]
            if not isinstance(value, expected_type):
                raise ReSimValidationError(
                    f"expected type {expected_type.__name__}, got {type(value).__name__} with value {format_value(value)}",
                    topic=topic_name,
                    field=field_name,
                )

    def _validate_data(self, topic_name: str, data: dict[str, Any]) -> None:
        """
        Validate the data dictionary against the topic's schema.

        Args:
            topic_name: The name of the topic.
            data: The data dictionary to validate.

        Raises:
            ReSimValidationError: If the topic is not found or data doesn't match schema.
        """
        # Skip validation if disabled
        if not self.validation_enabled:
            return

        # Check if the topic exists
        if topic_name not in self.topics:
            available_topics = ", ".join(f"'{t}'" for t in self.topics.keys())
            raise ReSimValidationError(
                f"topic '{topic_name}' not found in configuration. Available topics: {available_topics}"
            )

        topic_config = self.topics[topic_name]

        # Check if the topic has a schema
        if "schema" not in topic_config:
            raise ReSimValidationError(
                f"topic '{topic_name}' does not have a schema defined in configuration"
            )

        schema = topic_config["schema"]

        # Validate each field in the data
        for field_name, value in data.items():
            if field_name not in schema:
                available_fields = ", ".join(f"'{f}'" for f in schema.keys())
                raise ReSimValidationError(
                    f"field '{field_name}' not found in schema. Available fields: {available_fields}",
                    topic=topic_name,
                    field=field_name,
                )

            expected_type_str = schema[field_name]
            self._validate_value(value, expected_type_str, field_name, topic_name)

        # Check for missing required fields
        missing_fields = [f for f in schema.keys() if f not in data]
        if missing_fields:
            missing_str = ", ".join(f"'{f}'" for f in missing_fields)
            provided_str = ", ".join(f"'{f}'" for f in data.keys())
            raise ReSimValidationError(
                f"missing required field(s): {missing_str}. Provided fields: {provided_str}",
                topic=topic_name,
            )

    def emit(
        self,
        topic_name: str,
        data: dict[str, Any],
        timestamp: Optional[Union[int, Timestamp]] = None,
    ) -> None:
        """
        Emit a single datapoint with optional timestamp.

        Args:
            topic_name: The name of the topic to emit the data to.
            data: A dictionary of data to emit.
            timestamp: Optional timestamp of the data point.

        Raises:
            ReSimValidationError: If the topic is not found or validation fails.
        """
        self._validate_data(topic_name, data)

        emit(
            topic_name=topic_name,
            data=data,
            timestamp=timestamp,
            file=self.file,
        )

    def emit_series(
        self,
        topic_name: str,
        data: dict[str, list[Any]],
        timestamps: Optional[
            Union[list[int], list[Timestamp], npt.NDArray[np.int_]]
        ] = None,
    ) -> None:
        """
        Emit a series of datapoints with optional timestamps.

        Args:
            topic_name: The name of the topic to emit the data to.
            data: A dictionary where each value is a list of data points.
            timestamps: Optional list of timestamps corresponding to each data point.
                       If not provided, emissions will have no timestamps.

        Raises:
            ReSimValidationError: If the topic is not found, validation fails, or array lengths don't match.
        """
        self._validate_series_data(topic_name, data, timestamps)

        emit(
            topic_name=topic_name,
            data=data,
            timestamps=timestamps,
            file=self.file,
        )

    def emit_event(
        self,
        topic_name: str,
        data: dict[str, Any],
        timestamp: Union[int, Timestamp],
    ) -> None:
        """
        Emit an event with a timestamp.

        Args:
            topic_name: The name of the event topic to emit to.
            data: A dictionary of event data.
            timestamp: The timestamp when the event occurred.

        Raises:
            ReSimValidationError: If the topic is not found, not configured as an event, or validation fails.
        """
        self._validate_event_topic(topic_name)
        self._validate_data(topic_name, data)

        emit(
            topic_name=topic_name,
            data=data,
            timestamp=timestamp,
            event=True,
            file=self.file,
        )
