# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import json
from io import TextIOWrapper
from pathlib import Path
from typing import Any, Optional, Union

import numpy as np
import numpy.typing as npt

from resim.metrics.python.metrics_utils import Timestamp


def emit(
    topic_name: str,
    data: dict[str, Any],
    *,
    timestamp: Optional[Union[int, Timestamp]] = None,
    timestamps: Optional[
        Union[list[int], list[Timestamp], npt.NDArray[np.int_]]
    ] = None,
    event: bool = False,
    file_path: Path = Path("/tmp/resim/outputs/emissions.ndjson"),
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
