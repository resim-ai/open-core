# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""This module contains a free function used to unpack a set of
metrics and metrics data protos corresponding to a single job into a
set of Metric and MetricsData objects referencing each other as
appropriate.
"""

import numpy as np
import uuid
from dataclasses import dataclass

from google.protobuf import timestamp_pb2
import resim.metrics.proto.metrics_pb2 as mp

from resim.metrics.python.metrics_utils import Timestamp
from resim.metrics.python.metrics import (
    ScalarMetric,
    BarChartMetric,
    LinePlotMetric,
    DoubleSummaryMetric,
    DoubleOverTimeMetric,
    HistogramMetric,
    StatesOverTimeMetric,
    GroupedMetricsData,
    SeriesMetricsData,
    Metric,
    MetricsData)


@dataclass
class UnpackedMetrics:
    metrics: list[Metric]
    metrics_data: list[MetricsData]
    names: set[str]


def unpack_metrics(*,
                   metrics: list[mp.Metric],
                   metrics_data: list[mp.MetricsData]) -> UnpackedMetrics:
    id_to_metrics_map = {
        uuid.UUID(
            metric.metric_id.id.data): metric for metric in metrics}
    id_to_metrics_data_map = {
        uuid.UUID(
            md.metrics_data_id.id.data): md for md in metrics_data}

    id_to_unpacked_metrics_data = {}

    def recursive_unpack_metrics_data(current_id: uuid.UUID) -> None:
        if current_id in id_to_unpacked_metrics_data:
            return
        metrics_data = id_to_metrics_data_map[current_id]

        if metrics_data.is_indexed:
            index_id = uuid.UUID(metrics_data.index_data_id.id.data)
            recursive_unpack_metrics_data(index_id)
            assert index_id in id_to_unpacked_metrics_data

        _unpack_metrics_data(metrics_data, id_to_unpacked_metrics_data)

    for metrics_data_id in id_to_metrics_data_map:
        recursive_unpack_metrics_data(metrics_data_id)

    # TODO remove
    assert len(id_to_unpacked_metrics_data) == len(id_to_metrics_data_map)

    for key, value in id_to_unpacked_metrics_data.items():
        if "Number of detections status" in value.name:
            print(value)

    # for key, val in id_to_unpacked_metrics_data.items():
    #    try:
    #        print(val.index_)
    #    except:
    #        #print(val.name)
    #        #print(val.unit)
    #        print("BAD", val.name)
    #        #print("BAD", type(val.index_data))

    # TODO
    return UnpackedMetrics(metrics=[], metrics_data=[], names=set())


def _unpack_timestamp(timestamp: timestamp_pb2.Timestamp):
    return Timestamp(secs=timestamp.seconds, nanos=timestamp.nanos)


def _unpack_metrics_data(metrics_data: mp.MetricsData,
                         id_to_unpacked_metrics_data: dict[uuid.UUID,
                                                           MetricsData]):
    data_id = uuid.UUID(metrics_data.metrics_data_id.id.data)
    if metrics_data.is_per_category:
        assert metrics_data.WhichOneof("data") == "series_per_category"

        category_to_series = {}
        for category in metrics_data.category_names:
            series_to_unpack = metrics_data.series_per_category.category_to_series[category]
            data_case = series_to_unpack.WhichOneof("series")
            if data_case == "doubles":
                series = np.array(
                    series_to_unpack.doubles.series,
                    dtype=np.float64)
            elif data_case == "timestamps":
                series = np.array([_unpack_timestamp(
                    ts) for ts in series_to_unpack.timestamps.series], dtype=Timestamp)
            elif data_case == "uuids":
                series = np.array(
                    [uuid.UUID(element.data) for element in series_to_unpack.uuids.series],
                    dtype=uuid.UUID)
            elif data_case == "strings":
                series = np.array(
                    series_to_unpack.strings.series, dtype=str)
            elif data_case == "statuses":
                series = np.array(
                    series_to_unpack.statuses.series, dtype=int)

            category_to_series[category] = series
            
        index = id_to_unpacked_metrics_data[uuid.UUID(
            metrics_data.index_data_id.id.data)] if metrics_data.is_indexed else None
        unpacked = GroupedMetricsData(name=metrics_data.name,
                                      category_to_series=category_to_series,
                                      unit=metrics_data.unit,
                                      index_data=index)

    else:
        assert metrics_data.WhichOneof("data") == "series"
        data_case = metrics_data.series.WhichOneof("series")
        if data_case == "doubles":
            series = np.array(
                metrics_data.series.doubles.series,
                dtype=np.float64)
        elif data_case == "timestamps":
            series = np.array(
                metrics_data.series.timestamps.series,
                dtype=Timestamp)
            series = np.array([_unpack_timestamp(
                ts) for ts in metrics_data.series.timestamps.series], dtype=Timestamp)
        elif data_case == "uuids":
            series = np.array(
                [uuid.UUID(element.data) for element in metrics_data.series.uuids.series],
                dtype=uuid.UUID)
        elif data_case == "strings":
            series = np.array(metrics_data.series.strings.series, dtype=str)
        elif data_case == "statuses":
            series = np.array(metrics_data.series.statuses.series, dtype=int)

        index = id_to_unpacked_metrics_data[uuid.UUID(
            metrics_data.index_data_id.id.data)] if metrics_data.is_indexed else None
        unpacked = SeriesMetricsData(name=metrics_data.name,
                                     series=series,
                                     unit=metrics_data.unit,
                                     index_data=index)
    unpacked.id = data_id

    id_to_unpacked_metrics_data[data_id] = unpacked
