from __future__ import annotations


import uuid
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from resim.metrics.proto import metrics_pb2
from resim.utils.proto import uuid_pb2
from google.protobuf import timestamp_pb2


@dataclass(repr=True)
class Timestamp:
    secs: int
    nanos: int

    def pack(self: Timestamp) -> timestamp_pb2.Timestamp:
        msg = timestamp_pb2.Timestamp()
        msg.seconds = self.secs
        msg.nanos = self.nanos
        return msg


@dataclass(repr=True)
class HistogramBucket:
    lower: float
    upper: float

    def pack(self: HistogramBucket) -> metrics_pb2.HistogramMetricValues.Bucket:
        msg = metrics_pb2.HistogramMetricValues.Bucket()
        msg.lower = self.lower
        msg.upper = self.upper
        return msg


@dataclass(repr=True)
class DoubleFailureDefinition:
    fails_above: Optional[float]
    fails_below: Optional[float]

    def pack(self: DoubleFailureDefinition) -> metrics_pb2.DoubleFailureDefinition:
        msg = metrics_pb2.DoubleFailureDefinition()
        if self.fails_above is not None:
            msg.fails_above = self.fails_above

        if self.fails_below is not None:
            msg.fails_below = self.fails_below

        return msg


def pack_uuid_to_proto(uuid_obj: uuid.UUID) -> uuid_pb2.UUID:
    uuid_msg = uuid_pb2.UUID()
    uuid_msg.data = str(uuid_obj)
    return uuid_msg


def pack_series_to_proto(series: np.ndarray, indexed: bool) -> Tuple[metrics_pb2.MetricsDataType, metrics_pb2.Series]:
    data_type, series_msg = metrics_pb2.MetricsDataType.Value(
        'NO_DATA_TYPE'), metrics_pb2.Series()

    if len(series) == 0:
        data_type = metrics_pb2.MetricsDataType.Value('NO_DATA_TYPE')
    elif isinstance(series[0], float):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'DOUBLE_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_DOUBLE_SERIES_DATA_TYPE')
        series_msg.doubles.series.extend(list(series))
    elif isinstance(series[0], Timestamp):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'TIMESTAMP_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_TIMESTAMP_SERIES_DATA_TYPE')
        series_msg.timestamps.series.extend([t.pack() for t in series])
    elif isinstance(series[0], uuid.UUID):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'UUID_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_UUID_SERIES_DATA_TYPE')
        series_msg.uuids.series.extend([pack_uuid_to_proto(i) for i in series])
    elif isinstance(series[0], str):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'STRING_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_STRING_SERIES_DATA_TYPE')
        series_msg.strings.series.extend(list(series))
    elif isinstance(series[0], np.int64):
        # TODO(tknowles): We assume all ints are metric statuses!
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'METRIC_STATUS_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_METRIC_STATUS_SERIES_DATA_TYPE')
        series_msg.statuses.series.extend(list(series))
    else:
        print(type(metrics_pb2.MetricStatus.Value('NO_METRIC_STATUS')))
        raise ValueError(
            f"Invalid data type packed to proto: {type(series[0])}")

    return data_type, series_msg
