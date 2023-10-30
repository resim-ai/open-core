from __future__ import annotations
import uuid

import numpy as np

from resim.metrics.proto import metrics_pb2
from resim.metrics.proto.metrics_pb2 import MetricStatus, MetricImportance
from resim.utils.proto import uuid_pb2
from google.protobuf import timestamp_pb2

from resim.metrics.python.metrics import *

random.seed(194842)

# Fake data, assume this is read in
NUM_ACTORS = 10
EXAMPLE_ID_SET = [uuid.uuid4() for i in range(NUM_ACTORS)]
EXAMPLE_COUNTS = [10 * (i + 1) for i in range(NUM_ACTORS)]
EXAMPLE_STATES_SET = ['CAR', 'TRUCK', 'BIKE',
                      'PEDESTRIAN', 'DEBRIS', 'UNKNOWN']
EXAMPLE_DETECTIONS_SET = ['CAR_VEHICLE', 'TRUCK_VEHICLE',
                          'BIKE_VEHICLE', 'PEDESTRIAN_VEHICLE', 'DEBRIS_VEHICLE', 'UNKNOWN']
EXAMPLE_FAILURE_STATES = ['UNKNOWN']
EXAMPLE_LEGEND_SERIES_NAMES = ['Labels', 'Detections']

EXAMPLE_IDS = np.array([EXAMPLE_ID_SET[i] for i in range(
    NUM_ACTORS) for _ in range(EXAMPLE_COUNTS[i])])
EXAMPLE_TIMESTAMPS = np.array(
    [Timestamp(secs=i + 1, nanos=0) for i, _ in enumerate(EXAMPLE_IDS)])
EXAMPLE_LABELS = np.array([random.choice(EXAMPLE_STATES_SET)
                          for _ in EXAMPLE_IDS])
EXAMPLE_DETECTIONS = np.array(
    [random.choice(EXAMPLE_DETECTIONS_SET) for _ in EXAMPLE_IDS])

FIRST_VEHICLE_MASK = (EXAMPLE_IDS == EXAMPLE_ID_SET[0])
FIRST_VEHICLE_IDS = EXAMPLE_IDS[FIRST_VEHICLE_MASK]
FIRST_VEHICLE_TIMESTAMPS = EXAMPLE_TIMESTAMPS[FIRST_VEHICLE_MASK]
FIRST_VEHICLE_LABELS = EXAMPLE_LABELS[FIRST_VEHICLE_MASK]
FIRST_VEHICLE_DETECTIONS = EXAMPLE_DETECTIONS[FIRST_VEHICLE_MASK]

JOB_ID = uuid.uuid4()

# Actual data code
metrics = ResimMetricsWriter(JOB_ID)
timestamp_data = (metrics
                  .add_series_metrics_data('Timestamps').
                  with_series(EXAMPLE_TIMESTAMPS).
                  with_unit('seconds'))
id_data = (metrics.
           add_series_metrics_data('Actor IDs').
           with_series([str(i) for i in EXAMPLE_IDS]).
           with_unit('UUID')
           .with_index_data(timestamp_data))
labels = (metrics.
          add_series_metrics_data('Labels')
          .with_series(EXAMPLE_LABELS)
          .with_unit('Category')
          .with_index_data(timestamp_data))
detections = (metrics
              .add_series_metrics_data('Detections')
              .with_series(EXAMPLE_DETECTIONS)
              .with_unit('Category')
              .with_index_data(timestamp_data))
remapped_detections = metrics.add_metrics_data(detections.map(
    lambda series, index: series[index] if not series[index].endswith(
        '_VEHICLE') else series[index][:-len('_VEHICLE')],
    'Remapped detections',
    detections.unit
))

# Example for a grouped metric, handling data fairly manually
grouped_labels = metrics.add_metrics_data(labels.group_by(id_data))
grouped_detections = metrics.add_metrics_data(
    remapped_detections.group_by(id_data))

grouped_detections.pack()

states_over_time_metric = metrics.add_states_over_time_metric(
    'Labels vs detections')

states_over_time_metric = (states_over_time_metric
                           .with_description('Plot of category labels vs detections, grouped by actor')
                           .with_states_over_time_data([grouped_labels, grouped_detections])
                           .with_legend_series_names(["Labels", "Detections"])
                           .with_blocking(True)
                           .with_should_display(True)
                           .with_status(MetricStatus.Value('NOT_APPLICABLE_METRIC_STATUS'))
                           .with_states_set(EXAMPLE_STATES_SET)
                           .with_importance(MetricImportance.Value("HIGH_IMPORTANCE"))
                           )

# Now retrospectively compute some statuses and add them
# TODO(tknowles): I ultimately want to provide a nicer way to compute/attach statuses to metrics,
# but I suspect it will resemble something like this.
label_statuses = metrics.add_metrics_data(grouped_labels.map(
    lambda s, i, c: MetricStatus.Value('NOT_APPLICABLE_METRIC_STATUS'),
    'Label pass/fail'
))

detections_statuses = metrics.add_metrics_data(grouped_detections.map(
    lambda series, index, cat: MetricStatus.Value(
        'PASSED_METRIC_STATUS') if series[index] == grouped_labels.category_to_series[cat][index] else MetricStatus.Value('FAILED_METRIC_STATUS'),
    'Detection pass/fail'
))

print('Before', states_over_time_metric.statuses_over_time_data)

states_over_time_metric = (
    states_over_time_metric
    .append_statuses_over_time_data(label_statuses)
    .append_statuses_over_time_data(detections_statuses)
)

print('After', states_over_time_metric.statuses_over_time_data)

# We can do a more simple metric in one chain
# states_over_time_simple_metric = (metrics
#                                   .add_states_over_time_metric("Simple labels vs detections"))

a = StatesOverTimeMetric(
    "Intermediate", statuses_over_time_data="Something else")

print(StatesOverTimeMetric("Test example"))

#   .with_description(f'Plot of category labels vs detections for actor {FIRST_VEHICLE_IDS[0]}')
#   .with_blocking(True)
#   .with_should_display(True)
#   .with_status(MetricStatus.Value('NOT_APPLICABLE_METRIC_STATUS'))
#   .with_states_set(EXAMPLE_STATES_SET)
#   .with_importance(MetricImportance.Value("HIGH_IMPORTANCE"))
#   .with_states_over_time_series(
#       states_over_time_series={
#           "Labels": FIRST_VEHICLE_LABELS,
#           "Detections": FIRST_VEHICLE_DETECTIONS
#       },
#       indices={
#           "Labels": FIRST_VEHICLE_TIMESTAMPS,
#           "Detections": FIRST_VEHICLE_TIMESTAMPS
#       }
#   )
#   .with_legend_series_names(["Labels", "Detections"]))

# print(states_over_time_simple_metric.statuses_over_time_data)
