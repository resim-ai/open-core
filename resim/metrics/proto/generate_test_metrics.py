# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# pylint: disable=too-many-statements,too-many-locals

"""
generate_test_metrics.py

This module creates a valid resim.metrics.proto.JobMetrics message containing
representative data for each of the metrics types so we can test our validation
code and other code that needs to operate on such data. It is intended for use
in tests only, and not in production code.
"""


import uuid
from typing import Any

import resim.metrics.proto.metrics_pb2 as mp


def _get_uuid_str() -> str:
    """Helper to get a valid UUID as a hex str"""
    return '{' + str(uuid.uuid4()) + '}'


def _add_double_summary_metric(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Maximum Speed"
    metric.type = mp.DOUBLE_SUMMARY_METRIC_TYPE
    metric.description = "Maximum speed reached by actor"
    metric.status = mp.PASSED_METRIC_STATUS
    metric.should_display = True
    metric.blocking = False
    metric.importance = mp.MEDIUM_IMPORTANCE
    metric.order = 1.0
    metric.job_id.CopyFrom(job_metrics.job_id)

    data = job_metrics.metrics_data.add()
    data.metrics_data_id.id.data = _get_uuid_str()
    data.data_type = mp.DOUBLE_SERIES_DATA_TYPE
    data.name = "Ad-hoc speed metric"
    data.unit = "m/s"
    data.is_per_category = False
    data.series.doubles.series.append(27.)

    status_data = job_metrics.metrics_data.add()
    status_data.metrics_data_id.id.data = _get_uuid_str()
    status_data.data_type = mp.METRIC_STATUS_SERIES_DATA_TYPE
    status_data.name = "Speed metric status"
    status_data.unit = ""
    status_data.is_per_category = False
    status_data.series.statuses.series.append(mp.PASSED_METRIC_STATUS)

    double_summary_values = metric.metric_values.double_metric_values
    double_summary_values.value_data_id.CopyFrom(data.metrics_data_id)
    double_summary_values.status_data_id.CopyFrom(status_data.metrics_data_id)
    double_summary_values.failure_definition.fails_below = 0.
    double_summary_values.failure_definition.fails_above = 29.0576


def _add_event_counts(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Event counts"
    metric.type = mp.DOUBLE_SUMMARY_METRIC_TYPE
    metric.description = "Counts for various events"
    metric.status = mp.PASSED_METRIC_STATUS
    metric.should_display = True
    metric.blocking = False
    metric.importance = mp.MEDIUM_IMPORTANCE
    metric.order = 2.0
    metric.job_id.CopyFrom(job_metrics.job_id)

    data = job_metrics.metrics_data.add()
    data.metrics_data_id.id.data = _get_uuid_str()
    data.data_type = mp.DOUBLE_SERIES_DATA_TYPE
    data.name = "Event counts data"
    data.unit = ""
    data.is_per_category = True
    data.category_names.append("Engage")
    data.category_names.append("Disengage")
    data.series_per_category.category_to_series["Engage"].doubles.series.append(
        4.)
    data.series_per_category.category_to_series["Disengage"].doubles.series.append(
        3.)

    status_data = job_metrics.metrics_data.add()
    status_data.metrics_data_id.id.data = _get_uuid_str()
    status_data.data_type = mp.METRIC_STATUS_SERIES_DATA_TYPE
    status_data.name = "Event counts status"
    status_data.unit = ""
    status_data.is_per_category = True
    status_data.category_names.append("Engage")
    status_data.category_names.append("Disengage")
    status_data.series_per_category.category_to_series["Engage"].statuses.series.append(
        mp.PASSED_METRIC_STATUS)
    status_data.series_per_category.category_to_series["Disengage"].statuses.series.append(
        mp.FAIL_BLOCK_METRIC_STATUS)

    double_summary_values = metric.metric_values.double_metric_values
    double_summary_values.value_data_id.CopyFrom(data.metrics_data_id)
    double_summary_values.status_data_id.CopyFrom(status_data.metrics_data_id)


def _add_subsystem_states(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Subsystem states"
    metric.type = mp.STATES_OVER_TIME_METRIC_TYPE
    metric.description = "States of subsystems over time"
    metric.status = mp.PASSED_METRIC_STATUS
    metric.should_display = True
    metric.blocking = True
    metric.importance = mp.CRITICAL_IMPORTANCE
    metric.order = 3.0
    metric.job_id.CopyFrom(job_metrics.job_id)

    times = job_metrics.metrics_data.add()
    times.metrics_data_id.id.data = _get_uuid_str()
    times.data_type = mp.TIMESTAMP_SERIES_DATA_TYPE
    times.name = "Subsystem state times"
    times.is_per_category = True
    times.category_names.extend(["Planner", "Localization"])
    planner_arr = times.series_per_category.category_to_series["Planner"]
    localization_arr = times.series_per_category.category_to_series["Localization"]

    series_length = 10
    for i in range(series_length):
        planner_arr.timestamps.series.add().seconds = i
        localization_arr.timestamps.series.add().seconds = i

    def create_data_series(*,
                          data_type: mp.MetricsDataType,
                          name: str,
                          series_getter: str,
                          planner_data: list[Any],
                          localization_data: list[Any]) -> mp.MetricsDataId:
        data = job_metrics.metrics_data.add()
        data.metrics_data_id.id.data = _get_uuid_str()
        data.data_type = data_type
        data.name = name
        data.is_per_category = True
        data.category_names.extend(["Planner", "Localization"])

        data.is_indexed = True
        data.index_data_id.CopyFrom(times.metrics_data_id)
        data.index_data_type = mp.TIMESTAMP_SERIES_DATA_TYPE

        planner_arr = data.series_per_category.category_to_series["Planner"]
        getattr(planner_arr, series_getter).series.extend(planner_data)
        localization_arr = data.series_per_category.category_to_series["Localization"]
        getattr(localization_arr, series_getter).series.extend(localization_data)
        return data.metrics_data_id

    value_id = create_data_series(
        data_type=mp.INDEXED_STRING_SERIES_DATA_TYPE,
        name="Subsystem states data",
        series_getter="strings",
        planner_data=[
            "ENGAGED" if (3 <= i < 7) else "DISENGAGED" for i in range(series_length)],
        localization_data=series_length *
        ["ENGAGED"])

    status_id = create_data_series(
        data_type=mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE,
        name="Subsystem states status",
        series_getter="statuses",
        planner_data=series_length * [mp.PASSED_METRIC_STATUS],
        localization_data=series_length * [mp.PASSED_METRIC_STATUS])

    states_over_time = metric.metric_values.states_over_time_metric_values
    states_over_time.states_over_time_data_id.add().CopyFrom(value_id)
    states_over_time.statuses_over_time_data_id.add().CopyFrom(
        status_id)
    states_over_time.states_set.extend(
        ["ENGAGED", "DISENGAGED", "FAULTED"])
    states_over_time.failure_states.extend(["FAULTED"])


def _add_double_over_time_metric(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Time to collision"
    metric.type = mp.DOUBLE_OVER_TIME_METRIC_TYPE
    metric.description = "Time to collide with leading actor"
    metric.status = mp.PASSED_METRIC_STATUS  # Switches to FAILED below
    metric.should_display = True
    metric.blocking = True
    metric.importance = mp.CRITICAL_IMPORTANCE
    metric.order = 4.0
    metric.job_id.CopyFrom(job_metrics.job_id)

    ttc_index = job_metrics.metrics_data.add()
    ttc_index.metrics_data_id.id.data = _get_uuid_str()
    ttc_index.data_type = mp.TIMESTAMP_SERIES_DATA_TYPE
    ttc_index.name = "TTC Times"
    ttc_index.unit = ""

    ttc_timestamps = ttc_index.series.timestamps
    for i in range(10):
        ttc_timestamps.series.add().seconds = i

    actor_ttc_data = [
        {
            "name": "Actor 1",
            "doubles": [2.4, 2.1, 1.9, 1.8, 1.9, 2.3, 2.6, 3.0, 4.1, 7.2],
        },
        {
            "name": "Actor 2",
            "doubles": [3.0, 2.9, 2.5, 2.0, 1.5, 1.6, 1.2, 0.9, 0.8, 1.1],
        }]

    double_over_time_values = metric.metric_values.double_over_time_metric_values
    fails_below = 1.0

    # Add a failure definition per actor
    double_over_time_values.failure_definition.add().fails_below = fails_below
    double_over_time_values.failure_definition.add().fails_below = fails_below

    double_over_time_values.start_time.seconds = 0
    double_over_time_values.end_time.seconds = 10
    double_over_time_values.y_axis_name = "TTC"

    actor_ttc_value_ids = []
    actor_ttc_status_ids = []
    for actor in actor_ttc_data:
        ttc = job_metrics.metrics_data.add()
        ttc.metrics_data_id.id.data = _get_uuid_str()
        actor_ttc_value_ids.append(ttc.metrics_data_id)
        ttc.data_type = mp.INDEXED_DOUBLE_SERIES_DATA_TYPE
        ttc.name = f"{actor['name']} TTC"
        ttc.unit = "sec"
        ttc.is_indexed = True
        ttc.index_data_id.CopyFrom(ttc_index.metrics_data_id)
        ttc.index_data_type = mp.TIMESTAMP_SERIES_DATA_TYPE

        ttc_doubles = ttc.series.doubles
        ttc_doubles.series.extend(actor["doubles"])

        ttc_status = job_metrics.metrics_data.add()
        ttc_status.metrics_data_id.id.data = _get_uuid_str()
        actor_ttc_status_ids.append(ttc_status.metrics_data_id)
        ttc_status.data_type = mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE
        ttc_status.name = f"{actor['name']} TTC Statuses"
        ttc_status.unit = ""
        ttc_status.is_indexed = True
        ttc_status.index_data_id.CopyFrom(ttc_index.metrics_data_id)
        ttc_status.index_data_type = mp.TIMESTAMP_SERIES_DATA_TYPE

        ttc_statuses = ttc_status.series.statuses
        for val in ttc_doubles.series:
            if val < fails_below:
                ttc_statuses.series.append(mp.FAIL_BLOCK_METRIC_STATUS)
                metric.status = mp.FAIL_BLOCK_METRIC_STATUS
            else:
                ttc_statuses.series.append(mp.PASSED_METRIC_STATUS)

        double_over_time_values.doubles_over_time_data_id.add().CopyFrom(
            ttc.metrics_data_id)
        double_over_time_values.statuses_over_time_data_id.add().CopyFrom(
            ttc_status.metrics_data_id)
    # Add some summary metrics too
    summary_metric = job_metrics.job_level_metrics.metrics.add()
    summary_metric.metric_id.id.data = _get_uuid_str()
    summary_metric.name = "Initial Speed"
    summary_metric.type = mp.DOUBLE_SUMMARY_METRIC_TYPE
    summary_metric.description = "The initial speed of actor 1"
    summary_metric.status = mp.PASSED_METRIC_STATUS
    summary_metric.should_display = True
    summary_metric.blocking = False
    summary_metric.importance = mp.ZERO_IMPORTANCE
    summary_metric.order = 5.0
    summary_metric.job_id.CopyFrom(job_metrics.job_id)
    values = summary_metric.metric_values.double_metric_values
    values.series_index = 0
    values.value_data_id.CopyFrom(actor_ttc_value_ids[0])
    values.status_data_id.CopyFrom(actor_ttc_status_ids[0])

    summary_metric = job_metrics.job_level_metrics.metrics.add()
    summary_metric.metric_id.id.data = _get_uuid_str()
    summary_metric.name = "Speed at five seconds"
    summary_metric.type = mp.DOUBLE_SUMMARY_METRIC_TYPE
    summary_metric.description = "The speed of actor 1 at time 5"
    summary_metric.status = mp.PASSED_METRIC_STATUS
    summary_metric.should_display = True
    summary_metric.blocking = False
    summary_metric.importance = mp.ZERO_IMPORTANCE
    summary_metric.order = 5.1
    summary_metric.job_id.CopyFrom(job_metrics.job_id)
    values = summary_metric.metric_values.double_metric_values
    values.timestamp_index.seconds = 5
    values.value_data_id.CopyFrom(actor_ttc_value_ids[0])
    values.status_data_id.CopyFrom(actor_ttc_status_ids[0])


def _add_line_plot_metric(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Distance prediction accuracy"
    metric.type = mp.LINE_PLOT_METRIC_TYPE
    metric.description = ("Plot of actual minimum distance of non-ego actors"
        "against predicted minimum distance")
    metric.status = mp.PASSED_METRIC_STATUS
    metric.should_display = True
    metric.blocking = False
    metric.importance = mp.LOW_IMPORTANCE
    metric.order = 1.2
    metric.job_id.CopyFrom(job_metrics.job_id)

    data_series = [
        {
            "name": "Min d",
            "data": [21.0, 2.0, 25.0, 15.5],
        }, {
            "name": "Min d predicted",
            "data": [19.2, 1.9, 26.7, 10.0],
        }]

    metrics_data_ids = []
    for series in data_series:
        metrics_data = job_metrics.metrics_data.add()
        metrics_data.metrics_data_id.id.data = _get_uuid_str()
        metrics_data.data_type = mp.DOUBLE_SERIES_DATA_TYPE
        metrics_data.name = series["name"]
        metrics_data.unit = "m/s"
        metrics_data.is_indexed = False
        metrics_data.series.doubles.series.extend(series["data"])
        metrics_data_ids.append(metrics_data.metrics_data_id)

    line_plot_data_statuses = job_metrics.metrics_data.add()
    line_plot_data_statuses.metrics_data_id.id.data = _get_uuid_str()
    line_plot_data_statuses.data_type = mp.METRIC_STATUS_SERIES_DATA_TYPE
    line_plot_data_statuses.name = "D pred statuses"
    line_plot_data_statuses.unit = ""
    line_plot_data_statuses.is_indexed = False
    line_plot_data_statuses.series.statuses.series.extend(
        len(data_series[0]["data"]) * [mp.PASSED_METRIC_STATUS])

    line_plot_values = metric.metric_values.line_plot_metric_values
    line_plot_values.x_doubles_data_id.add().CopyFrom(metrics_data_ids[0])
    line_plot_values.y_doubles_data_id.add().CopyFrom(metrics_data_ids[1])
    line_plot_values.statuses_data_id.add().CopyFrom(
        line_plot_data_statuses.metrics_data_id)
    line_plot_values.x_axis_name = "Actual minimum distance"
    line_plot_values.x_axis_name = "Predicted minimum distance"


def _add_bar_chart_metric(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Detection latency"
    metric.type = mp.BAR_CHART_METRIC_TYPE
    metric.description = "Average latency on computing detections from images"
    metric.status = mp.FAIL_BLOCK_METRIC_STATUS
    metric.should_display = True
    metric.blocking = False
    metric.importance = mp.MEDIUM_IMPORTANCE
    metric.order = 6.0
    metric.job_id.CopyFrom(job_metrics.job_id)

    labels = job_metrics.metrics_data.add()
    labels.name = "Detection latency labels"
    labels.metrics_data_id.id.data = _get_uuid_str()
    labels.data_type = mp.STRING_SERIES_DATA_TYPE
    labels.is_indexed = False
    labels.series.strings.series.extend(
        [f"camera_{i}" for i in range(3)])

    data_series: list[dict[str, Any]] = [{
        "name": "Camera timings",
        "data": [1.5, 2.6, 1.8],
        "statuses": [mp.PASSED_METRIC_STATUS, mp.FAIL_BLOCK_METRIC_STATUS,
                     mp.PASSED_METRIC_STATUS],
    },
        {
        "name": "Pytorch timings",
        "data": [10.1, 9.2, 12.3],
        "statuses": [mp.FAIL_BLOCK_METRIC_STATUS, mp.FAIL_BLOCK_METRIC_STATUS,
                     mp.PASSED_METRIC_STATUS],
    }
    ]

    bar_chart_values = metric.metric_values.bar_chart_metric_values

    for series in data_series:
        metrics_data = job_metrics.metrics_data.add()
        metrics_data.metrics_data_id.id.data = _get_uuid_str()
        metrics_data.data_type = mp.INDEXED_DOUBLE_SERIES_DATA_TYPE
        metrics_data.name = series["name"]
        metrics_data.unit = "msec"
        metrics_data.is_indexed = True
        metrics_data.index_data_id.CopyFrom(labels.metrics_data_id)
        metrics_data.index_data_type = mp.STRING_SERIES_DATA_TYPE
        metrics_data.series.doubles.series.extend(series["data"])

        metrics_data_statuses = job_metrics.metrics_data.add()
        metrics_data_statuses.metrics_data_id.id.data = _get_uuid_str()
        metrics_data_statuses.data_type = mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE
        metrics_data_statuses.name = series["name"] + " statuses"
        metrics_data_statuses.unit = ""
        metrics_data_statuses.is_indexed = True
        metrics_data_statuses.index_data_id.CopyFrom(labels.metrics_data_id)
        metrics_data_statuses.index_data_type = mp.STRING_SERIES_DATA_TYPE
        metrics_data_statuses.series.statuses.series.extend(series["statuses"])
        bar_chart_values.values_data_id.add().CopyFrom(metrics_data.metrics_data_id)
        bar_chart_values.statuses_data_id.add().CopyFrom(
            metrics_data_statuses.metrics_data_id)

    bar_chart_values.legend_series_names.extend(
        ["Camera timings", "Pytorch timings"])
    bar_chart_values.x_axis_name = "Component"
    bar_chart_values.y_axis_name = "Mean Latency"


def _add_string_and_uuid_summary_metrics(job_metrics: mp.JobMetrics) -> None:
    temperature_probe_locations = job_metrics.metrics_data.add()
    temperature_probe_locations.metrics_data_id.id.data = _get_uuid_str()
    temperature_probe_locations.data_type = mp.STRING_SERIES_DATA_TYPE
    temperature_probe_locations.name = "Temperature probe locations"
    temperature_probe_locations.unit = ""
    temperature_probe_locations.series.strings.series.extend(
        ["Lower Chassis", "Upper Chassis", "Sensors"])

    temperature_data = job_metrics.metrics_data.add()
    temperature_data.metrics_data_id.id.data = _get_uuid_str()
    temperature_data.data_type = mp.INDEXED_DOUBLE_SERIES_DATA_TYPE
    temperature_data.name = "Temperatures"
    temperature_data.unit = "C"
    temperature_data.is_indexed = True
    temperature_data.index_data_id.CopyFrom(
        temperature_probe_locations.metrics_data_id)
    temperature_data.index_data_type = mp.STRING_SERIES_DATA_TYPE
    temperature_data.series.doubles.series.extend([24., 25., 24.])

    temperature_status = job_metrics.metrics_data.add()
    temperature_status.metrics_data_id.id.data = _get_uuid_str()
    temperature_status.data_type = mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE
    temperature_status.name = "Temperature Statuses"
    temperature_status.unit = ""
    temperature_status.is_indexed = True
    temperature_status.index_data_id.CopyFrom(
        temperature_probe_locations.metrics_data_id)
    temperature_status.index_data_type = mp.STRING_SERIES_DATA_TYPE
    temperature_status.series.statuses.series.extend(
        3 * [mp.PASSED_METRIC_STATUS])

    temperature_metric = job_metrics.job_level_metrics.metrics.add()
    temperature_metric.metric_id.id.data = _get_uuid_str()
    temperature_metric.name = "Upper Chassis Temperature"
    temperature_metric.description = "The temperature of the upper chassis in Celsius"
    temperature_metric.type = mp.DOUBLE_SUMMARY_METRIC_TYPE
    temperature_metric.status = mp.PASSED_METRIC_STATUS
    temperature_metric.should_display = True
    temperature_metric.blocking = False
    temperature_metric.importance = mp.ZERO_IMPORTANCE
    temperature_metric.order = 7.0
    temperature_metric.job_id.CopyFrom(job_metrics.job_id)
    values = temperature_metric.metric_values.double_metric_values
    values.value_data_id.CopyFrom(temperature_data.metrics_data_id)
    values.status_data_id.CopyFrom(temperature_status.metrics_data_id)
    values.string_index = "Upper Chassis"

    temperature_data = job_metrics.metrics_data.add()
    temperature_data.metrics_data_id.id.data = _get_uuid_str()
    temperature_data.data_type = mp.INDEXED_DOUBLE_SERIES_DATA_TYPE
    temperature_data.name = "Temperatures 2"
    temperature_data.unit = "C"
    temperature_data.is_indexed = True
    temperature_data.index_data_id.CopyFrom(
        temperature_probe_locations.metrics_data_id)
    temperature_data.index_data_type = mp.STRING_SERIES_DATA_TYPE
    temperature_data.series.doubles.series.extend([24., 25., 24.])

    temperature_status = job_metrics.metrics_data.add()
    temperature_status.metrics_data_id.id.data = _get_uuid_str()
    temperature_status.data_type = mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE
    temperature_status.name = "Temperature Statuses 2"
    temperature_status.unit = ""
    temperature_status.is_indexed = True
    temperature_status.index_data_id.CopyFrom(
        temperature_probe_locations.metrics_data_id)
    temperature_status.index_data_type = mp.STRING_SERIES_DATA_TYPE
    temperature_status.series.statuses.series.extend(
        3 * [mp.PASSED_METRIC_STATUS])

    temperature_metric = job_metrics.job_level_metrics.metrics.add()
    temperature_metric.metric_id.id.data = _get_uuid_str()
    temperature_metric.name = "Upper Chassis Temperature 2"
    temperature_metric.description = "The temperature of the upper chassis in Celsius"
    temperature_metric.type = mp.DOUBLE_SUMMARY_METRIC_TYPE
    temperature_metric.status = mp.PASSED_METRIC_STATUS
    temperature_metric.should_display = True
    temperature_metric.blocking = False
    temperature_metric.importance = mp.ZERO_IMPORTANCE
    temperature_metric.order = 8.0
    temperature_metric.job_id.CopyFrom(job_metrics.job_id)
    values = temperature_metric.metric_values.double_metric_values
    values.value_data_id.CopyFrom(temperature_data.metrics_data_id)
    values.status_data_id.CopyFrom(temperature_status.metrics_data_id)
    values.string_index = "Upper Chassis"

    waypoints = job_metrics.metrics_data.add()
    waypoints.metrics_data_id.id.data = _get_uuid_str()
    waypoints.data_type = mp.UUID_SERIES_DATA_TYPE
    waypoints.name = "Mapped waypoints"
    waypoints.series.uuids.series.add().data = _get_uuid_str()
    waypoints.series.uuids.series.add().data = _get_uuid_str()
    waypoints.series.uuids.series.add().data = _get_uuid_str()

    distance_to_waypoints = job_metrics.metrics_data.add()
    distance_to_waypoints.metrics_data_id.id.data = _get_uuid_str()
    distance_to_waypoints.data_type = mp.INDEXED_DOUBLE_SERIES_DATA_TYPE
    distance_to_waypoints.name = "Distance to mapped waypoints"
    distance_to_waypoints.unit = "m"
    distance_to_waypoints.is_indexed = True
    distance_to_waypoints.index_data_id.CopyFrom(waypoints.metrics_data_id)
    distance_to_waypoints.index_data_type = mp.UUID_SERIES_DATA_TYPE
    distance_to_waypoints.series.doubles.series.extend([10., 20., 15.])

    distance_to_waypoints_status = job_metrics.metrics_data.add()
    distance_to_waypoints_status.metrics_data_id.id.data = _get_uuid_str()
    distance_to_waypoints_status.data_type = mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE
    distance_to_waypoints_status.name = "Distance to mapped waypoints status"
    distance_to_waypoints_status.unit = ""
    distance_to_waypoints_status.is_indexed = True
    distance_to_waypoints_status.index_data_id.CopyFrom(
        waypoints.metrics_data_id)
    distance_to_waypoints_status.index_data_type = mp.UUID_SERIES_DATA_TYPE
    distance_to_waypoints_status.series.statuses.series.extend(
        3 * [mp.PASSED_METRIC_STATUS])

    waypoint_distance_metric = job_metrics.job_level_metrics.metrics.add()
    waypoint_distance_metric.metric_id.id.data = _get_uuid_str()
    waypoint_distance_metric.name = "Distance to given waypoint"
    waypoint_distance_metric.description = "The distance to a specific waypoint"
    waypoint_distance_metric.type = mp.DOUBLE_SUMMARY_METRIC_TYPE
    waypoint_distance_metric.status = mp.PASSED_METRIC_STATUS
    waypoint_distance_metric.should_display = True
    waypoint_distance_metric.blocking = False
    waypoint_distance_metric.importance = mp.ZERO_IMPORTANCE
    waypoint_distance_metric.order = 9.0
    waypoint_distance_metric.job_id.CopyFrom(job_metrics.job_id)
    values = waypoint_distance_metric.metric_values.double_metric_values
    values.value_data_id.CopyFrom(distance_to_waypoints.metrics_data_id)
    values.status_data_id.CopyFrom(
        distance_to_waypoints_status.metrics_data_id)
    values.uuid_index.CopyFrom(waypoints.series.uuids.series[0])


def _add_states_over_time_metric(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Lane change states"
    metric.type = mp.STATES_OVER_TIME_METRIC_TYPE
    metric.description = "Lane change states over time"
    metric.status = mp.PASSED_METRIC_STATUS
    metric.should_display = False
    metric.blocking = False
    metric.importance = mp.ZERO_IMPORTANCE
    metric.order = 10.0
    metric.job_id.CopyFrom(job_metrics.job_id)

    times = job_metrics.metrics_data.add()
    times.metrics_data_id.id.data = _get_uuid_str()
    times.data_type = mp.TIMESTAMP_SERIES_DATA_TYPE
    times.name = "State Times"
    times.unit = ""
    for i in range(10):
        times.series.timestamps.series.add().seconds = i

    states = job_metrics.metrics_data.add()
    states.metrics_data_id.id.data = _get_uuid_str()
    states.data_type = mp.INDEXED_STRING_SERIES_DATA_TYPE
    states.name = "Actor 2 state data"
    states.unit = ""
    states.is_indexed = True
    states.index_data_id.CopyFrom(times.metrics_data_id)
    states.index_data_type = mp.TIMESTAMP_SERIES_DATA_TYPE
    states.series.strings.series.extend([
        "LANE_CHANGE" if (3 <= i < 7) else "LANE_FOLLOW" for i in range(10)])

    states_status = job_metrics.metrics_data.add()
    states_status.metrics_data_id.id.data = _get_uuid_str()
    states_status.data_type = mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE
    states_status.name = "Actor 2 state statuses"
    states_status.unit = ""
    states_status.is_indexed = True
    states_status.index_data_id.CopyFrom(times.metrics_data_id)
    states_status.index_data_type = mp.TIMESTAMP_SERIES_DATA_TYPE
    states_status.series.statuses.series.extend(10 * [mp.PASSED_METRIC_STATUS])

    states_over_time = metric.metric_values.states_over_time_metric_values
    states_over_time.states_over_time_data_id.add().CopyFrom(states.metrics_data_id)
    states_over_time.statuses_over_time_data_id.add().CopyFrom(
        states_status.metrics_data_id)
    states_over_time.states_set.extend(
        ["LANE_CHANGE", "LANE_FOLLOW", "COLLISION", "NO_STATE"])
    states_over_time.failure_states.extend(["COLLISION"])


def _add_histogram_metric(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Number of detections"
    metric.type = mp.HISTOGRAM_METRIC_TYPE
    metric.description = "Number of detections on each cycle"
    metric.status = mp.PASSED_METRIC_STATUS
    metric.should_display = True
    metric.blocking = False
    metric.importance = mp.ZERO_IMPORTANCE
    metric.order = 11.0
    metric.job_id.CopyFrom(job_metrics.job_id)

    data = job_metrics.metrics_data.add()
    data.metrics_data_id.id.data = _get_uuid_str()
    data.data_type = mp.DOUBLE_SERIES_DATA_TYPE
    data.name = "Number of detections data"
    data.unit = ""
    data.is_indexed = True
    data_values = [84, 58, 31, 17, 54, 19, 78, 38, 40, 58, 47, 44, 55, 4, 33,
                   32, 6, 65, 32, 10, 28, 35, 40, 80, 63, 61, 27, 50, 69, 27, 1,
                   40, 34, 53, 55, 66, 47, 30, 63, 46, 76, 65, 91, 64, 18, 38,
                   66, 40, 33, 43, 32, 73, 44, 39, 77, 0, 58, 50, 63, 63, 75,
                   58, 43, 46, 54, 66, 47, 55, 24, 71, 18, 31, 22, 56, 12, 62,
                   52, 56, 21, 70, 44, 52, 58, 60, 31, 53, 28, 50, 61, 72, 46,
                   65, 53, 21, 44, 55, 91, 92, 27, 23]
    data.series.doubles.series.extend(data_values)

    index = job_metrics.metrics_data.add()
    index.metrics_data_id.id.data = _get_uuid_str()
    index.data_type = mp.UUID_SERIES_DATA_TYPE
    index.name = "Image ID"
    index.unit = ""
    index.is_indexed = False
    for _ in range(len(data_values)):
        index.series.uuids.series.add().data = _get_uuid_str()

    data.index_data_id.CopyFrom(index.metrics_data_id)
    data.index_data_type = mp.UUID_SERIES_DATA_TYPE

    data_status = job_metrics.metrics_data.add()
    data_status.metrics_data_id.id.data = _get_uuid_str()
    data_status.data_type = mp.METRIC_STATUS_SERIES_DATA_TYPE
    data_status.name = "Number of detections status"
    data_status.unit = ""
    data_status.is_indexed = True
    data_status.index_data_id.CopyFrom(index.metrics_data_id)
    data_status.index_data_type = mp.UUID_SERIES_DATA_TYPE
    data_status.series.statuses.series.extend(
        len(data_values) * [mp.NOT_APPLICABLE_METRIC_STATUS])

    histogram_values = metric.metric_values.histogram_metric_values
    histogram_values.values_data_id.CopyFrom(data.metrics_data_id)
    histogram_values.statuses_data_id.CopyFrom(data_status.metrics_data_id)
    histogram_values.lower_bound = 0.0
    histogram_values.upper_bound = 100.0
    histogram_values.x_axis_name = "Number of detections"
    for i in range(10):
        bucket = histogram_values.buckets.add()
        bucket.lower = 10 * i
        bucket.upper = 10 * (i + 1)


def _add_scalar_metric(job_metrics: mp.JobMetrics) -> None:
    metric = job_metrics.job_level_metrics.metrics.add()
    metric.metric_id.id.data = _get_uuid_str()
    metric.name = "Ambient Temperature"
    metric.type = mp.SCALAR_METRIC_TYPE
    metric.description = "The temperature outside in Celsius"
    metric.status = mp.NOT_APPLICABLE_METRIC_STATUS
    metric.should_display = True
    metric.blocking = False
    metric.importance = mp.ZERO_IMPORTANCE
    metric.order = 12.0
    metric.job_id.CopyFrom(job_metrics.job_id)
    metric.metric_values.scalar_metric_values.value = 6.28318530718


def _populate_metrics_statuses(job_metrics: mp.JobMetrics) -> None:
    collection = job_metrics.job_level_metrics

    job_metrics_status = mp.PASSED_METRIC_STATUS
    for metric in collection.metrics:
        if metric.status == mp.FAIL_BLOCK_METRIC_STATUS and metric.blocking:
            job_metrics_status = mp.FAIL_BLOCK_METRIC_STATUS

    job_metrics.metrics_status = job_metrics_status
    job_metrics.job_level_metrics.metrics_status = job_metrics_status


def generate_test_metrics() -> mp.JobMetrics:
    """
    Generate a set of test metrics containing representative examples for each
    of our metric types.
    """
    job_metrics = mp.JobMetrics()
    job_metrics.job_id.id.data = _get_uuid_str()

    _add_double_summary_metric(job_metrics)
    _add_event_counts(job_metrics)
    _add_double_over_time_metric(job_metrics)
    _add_line_plot_metric(job_metrics)
    _add_bar_chart_metric(job_metrics)
    _add_states_over_time_metric(job_metrics)
    _add_histogram_metric(job_metrics)
    _add_scalar_metric(job_metrics)
    _add_subsystem_states(job_metrics)
    _add_string_and_uuid_summary_metrics(job_metrics)

    _populate_metrics_statuses(job_metrics)

    return job_metrics
