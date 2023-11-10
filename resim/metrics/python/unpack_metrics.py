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

import uuid
from dataclasses import dataclass

import resim.metrics.proto.metrics_pb2 as mp

# Just a stub for now
@dataclass
class Metric:
    ...

@dataclass
class MetricsData:
    ...

@dataclass
class UnpackedMetrics:
    metrics: dict[uuid.UUID, Metric]
    metrics_data: dict[uuid.UUID, MetricsData]
    names: set[str]


def unpack_metrics(metrics: list[mp.Metric],
                   metrics_data: list[mp.MetricsData]) -> UnpackedMetrics:
    ...
