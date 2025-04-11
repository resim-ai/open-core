# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from __future__ import annotations

import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Set, Tuple

import google.protobuf.timestamp_pb2 as timestamp_proto
import numpy as np
import resim.utils.proto.uuid_pb2 as uuid_proto

import resim.metrics.proto.metrics_pb2 as metrics_proto


class MetricStatus(Enum):
    NO_METRIC_STATUS = 0
    PASSED_METRIC_STATUS = 1
    FAIL_WARN_METRIC_STATUS = 2
    NOT_APPLICABLE_METRIC_STATUS = 3
    RAW_METRIC_STATUS = 4
    FAIL_BLOCK_METRIC_STATUS = 5


class MetricImportance(Enum):
    NO_SPECIFIED_IMPORTANCE = 0
    ZERO_IMPORTANCE = 1
    LOW_IMPORTANCE = 2
    MEDIUM_IMPORTANCE = 3
    HIGH_IMPORTANCE = 4
    CRITICAL_IMPORTANCE = 5


class TimestampType(Enum):
    NO_TYPE = 0
    ABSOLUTE_TIMESTAMP = 1
    RELATIVE_TIMESTAMP = 2


@dataclass(init=False, repr=True)
class ResimMetricsOutput:
    metrics_msg: metrics_proto.JobMetrics
    packed_ids: Set[uuid.UUID]

    def __init__(self) -> None:
        self.metrics_msg = metrics_proto.JobMetrics()
        self.packed_ids = set()


@dataclass(repr=True, order=True)
class Timestamp:
    secs: int
    nanos: int

    def to_nanos(self) -> int:
        """Convert the timestamp to number of nanoseconds."""
        return self.secs * 1_000_000_000 + self.nanos

    def pack(self: Timestamp) -> timestamp_proto.Timestamp:
        msg = timestamp_proto.Timestamp()
        msg.seconds = self.secs
        msg.nanos = self.nanos
        return msg

    @classmethod
    def unpack(cls, msg: timestamp_proto.Timestamp) -> Timestamp:
        return Timestamp(
            secs=msg.seconds,
            nanos=msg.nanos,
        )


@dataclass(repr=True)
class Tag:
    key: str
    value: str

    def pack(self: Tag) -> metrics_proto.Tag:
        tag = metrics_proto.Tag()
        tag.key = self.key
        tag.value = self.value
        return tag

    @classmethod
    def unpack(cls, msg: metrics_proto.Tag) -> Tag:
        return Tag(key=msg.key, value=msg.value)


@dataclass(repr=True)
class HistogramBucket:
    lower: float
    upper: float

    def pack(self: HistogramBucket) -> metrics_proto.HistogramMetricValues.Bucket:
        msg = metrics_proto.HistogramMetricValues.Bucket()
        msg.lower = self.lower
        msg.upper = self.upper
        return msg


@dataclass(repr=True)
class DoubleFailureDefinition:
    fails_above: Optional[float]
    fails_below: Optional[float]

    def pack(self: DoubleFailureDefinition) -> metrics_proto.DoubleFailureDefinition:
        msg = metrics_proto.DoubleFailureDefinition()
        if self.fails_above is not None:
            msg.fails_above = self.fails_above

        if self.fails_below is not None:
            msg.fails_below = self.fails_below
        return msg


def pack_uuid_to_proto(uuid_obj: uuid.UUID) -> uuid_proto.UUID:
    uuid_msg = uuid_proto.UUID()
    uuid_msg.data = str(uuid_obj)
    return uuid_msg


def pack_uuid_to_metric_id(uuid_obj: uuid.UUID) -> metrics_proto.MetricID:
    metric_id = metrics_proto.MetricId()
    metric_id.id.CopyFrom(pack_uuid_to_proto(uuid_obj))
    return metric_id


def pack_series_to_proto(
    series: np.ndarray, indexed: bool
) -> Tuple[metrics_proto.MetricsDataType, metrics_proto.Series]:
    data_type, series_msg = (
        metrics_proto.MetricsDataType.Value("NO_DATA_TYPE"),
        metrics_proto.Series(),
    )

    if len(series) == 0:
        data_type = metrics_proto.MetricsDataType.Value("NO_DATA_TYPE")
    elif isinstance(series[0], float):
        if not indexed:
            data_type = metrics_proto.MetricsDataType.Value("DOUBLE_SERIES_DATA_TYPE")
        else:
            data_type = metrics_proto.MetricsDataType.Value(
                "INDEXED_DOUBLE_SERIES_DATA_TYPE"
            )
        series_msg.doubles.series.extend(list(series))
    elif isinstance(series[0], Timestamp):
        if not indexed:
            data_type = metrics_proto.MetricsDataType.Value(
                "TIMESTAMP_SERIES_DATA_TYPE"
            )
        else:
            data_type = metrics_proto.MetricsDataType.Value(
                "INDEXED_TIMESTAMP_SERIES_DATA_TYPE"
            )
        series_msg.timestamps.series.extend([t.pack() for t in series])
    elif isinstance(series[0], uuid.UUID):
        if not indexed:
            data_type = metrics_proto.MetricsDataType.Value("UUID_SERIES_DATA_TYPE")
        else:
            data_type = metrics_proto.MetricsDataType.Value(
                "INDEXED_UUID_SERIES_DATA_TYPE"
            )
        series_msg.uuids.series.extend([pack_uuid_to_proto(i) for i in series])
    elif isinstance(series[0], str):
        if not indexed:
            data_type = metrics_proto.MetricsDataType.Value("STRING_SERIES_DATA_TYPE")
        else:
            data_type = metrics_proto.MetricsDataType.Value(
                "INDEXED_STRING_SERIES_DATA_TYPE"
            )
        series_msg.strings.series.extend(list(series))
    elif isinstance(series[0], MetricStatus):
        if not indexed:
            data_type = metrics_proto.MetricsDataType.Value(
                "METRIC_STATUS_SERIES_DATA_TYPE"
            )
        else:
            data_type = metrics_proto.MetricsDataType.Value(
                "INDEXED_METRIC_STATUS_SERIES_DATA_TYPE"
            )
        series_msg.statuses.series.extend([s.value for s in series])
    else:
        raise ValueError(f"Invalid data type packed to proto: {type(series[0])}")

    return data_type, series_msg
