# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit tests for metrics.py.
"""

import copy
import uuid
import unittest
from typing import cast, Any

import numpy as np

from google.protobuf.struct_pb2 import Struct
from resim.metrics.python import metrics, metrics_utils
from resim.metrics.python.metrics_utils import MetricStatus, MetricImportance
import resim.metrics.proto.metrics_pb2 as mp

# pylint: disable=too-many-public-methods


class MetricsTest(unittest.TestCase):
    def test_metric_eq(self) -> None:
        # SETUP
        job_id = uuid.uuid4()
        metric = metrics.ScalarMetric(
            "test_metric",
            "A metric for testing",
            MetricStatus.PASSED_METRIC_STATUS,
            MetricImportance.ZERO_IMPORTANCE,
            should_display=True,
            blocking=False,
            parent_job_id=job_id,
            order=None,
            value=24.0,
            failure_definition=None,
            unit="")

        # Test equality
        self.assertEqual(metric, metric)

        # Different type
        self.assertNotEqual(metric, 3)

        # Non-existent id
        metric_with_none_id = copy.copy(metric)
        metric_with_none_id.id = cast(uuid.UUID, None)
        with self.assertRaises(AssertionError):
            _ = metric_with_none_id == metric
        with self.assertRaises(AssertionError):
            _ = metric == metric_with_none_id

        # Different id
        metric_with_diff_id = copy.copy(metric)
        metric_with_diff_id.id = uuid.uuid4()
        self.assertNotEqual(metric_with_diff_id, metric)
        self.assertNotEqual(metric, metric_with_diff_id)

    def test_metric_setters(self) -> None:
        metric = metrics.ScalarMetric(name="test_metric")
        DESCRIPTION = "my metric"
        self.assertEqual(metric, metric.with_description(DESCRIPTION))
        self.assertEqual(metric.description, DESCRIPTION)

        self.assertEqual(metric, metric.with_status(MetricStatus.PASSED_METRIC_STATUS))
        self.assertEqual(metric.status, MetricStatus.PASSED_METRIC_STATUS)

        self.assertEqual(metric, metric.with_importance(MetricImportance.ZERO_IMPORTANCE))
        self.assertEqual(metric.importance, MetricImportance.ZERO_IMPORTANCE)

        self.assertEqual(metric, metric.with_should_display(True))
        self.assertTrue(metric.should_display)

        self.assertEqual(metric, metric.with_blocking(True))
        self.assertTrue(metric.blocking)

        self.assertEqual(metric, metric.with_event_metric(True))
        self.assertTrue(metric.event_metric)

    def assert_common_fields_match(self, *,
                                   msg: mp.Metric,
                                   metric: metrics.Metric) -> None:
        self.assertEqual(uuid.UUID(msg.metric_id.id.data), metric.id)
        self.assertEqual(msg.name, metric.name)
        self.assertEqual(msg.description, metric.description)
        self.assertEqual(msg.status, getattr(metric.status, "value"))
        self.assertEqual(msg.importance, getattr(metric.importance, "value"))
        self.assertEqual(msg.should_display, metric.should_display)
        self.assertEqual(msg.blocking, metric.blocking)
        self.assertEqual(uuid.UUID(msg.job_id.id.data), metric.parent_job_id)
        self.assertEqual(msg.order, metric.order)

    def test_metric_pack(self) -> None:
        # SETUP
        job_id = uuid.uuid4()
        metric = metrics.ScalarMetric(
            "test_metric",
            "A metric for testing",
            MetricStatus.PASSED_METRIC_STATUS,
            MetricImportance.ZERO_IMPORTANCE,
            should_display=True,
            blocking=False,
            parent_job_id=job_id,
            order=1.5,
            value=24.0,
            failure_definition=None,
            unit="")

        msg = metric.pack()
        self.assert_common_fields_match(msg=msg, metric=metric)

        optional_attr_list = [
            ("description", "description"),
            ("status", "status"),
            ("importance", "importance"),
            ("should_display", "should_display"),
            ("blocking", "blocking"),
            ("parent_job_id", "job_id"),
            ("order", "order")]

        default_values = mp.Metric()
        for unpacked_attr, packed_attr in optional_attr_list:
            modified_metric = copy.copy(metric)
            setattr(modified_metric, unpacked_attr, None)
            msg = modified_metric.pack()
            self.assertEqual(
                getattr(
                    msg, packed_attr), getattr(
                    default_values, packed_attr))

        with self.assertRaises(NotImplementedError):
            output = metrics_utils.ResimMetricsOutput()
            metrics.Metric.recursively_pack_into(metric, output)

    def test_unpack_common_fields(self) -> None:
        msg = mp.Metric()
        msg.metric_id.id.data = str(uuid.uuid4())
        msg.name = "test metric"
        msg.type = mp.MetricType.Value('DOUBLE_OVER_TIME_METRIC_TYPE')
        msg.description = "This is a test metric"
        msg.status = mp.MetricStatus.Value('PASSED_METRIC_STATUS')
        msg.should_display = True
        msg.blocking = False
        msg.importance = mp.MetricImportance.Value('ZERO_IMPORTANCE')
        msg.job_id.id.data = str(uuid.uuid4())
        msg.order = 0.5

        unpacked = metrics.Metric.unpack_common_fields(msg)
        self.assert_common_fields_match(msg=msg, metric=unpacked)
        optional_attr_list = [
            ("should_display", "should_display"),
            ("blocking", "blocking"),
            ("parent_job_id", "job_id"),
            ("order", "order")]

        for unpacked_attr, packed_attr in optional_attr_list:
            modified_msg = copy.copy(msg)
            modified_msg.ClearField(packed_attr)
            unpacked = metrics.Metric.unpack_common_fields(modified_msg)
            self.assertIs(getattr(unpacked, unpacked_attr), None)

        metric_type_list = [
            (mp.MetricType.Value('DOUBLE_OVER_TIME_METRIC_TYPE'), metrics.DoubleOverTimeMetric),
            (mp.MetricType.Value('LINE_PLOT_METRIC_TYPE'), metrics.LinePlotMetric),
            (mp.MetricType.Value('BAR_CHART_METRIC_TYPE'), metrics.BarChartMetric),
            (mp.MetricType.Value('STATES_OVER_TIME_METRIC_TYPE'), metrics.StatesOverTimeMetric),
            (mp.MetricType.Value('HISTOGRAM_METRIC_TYPE'), metrics.HistogramMetric),
            (mp.MetricType.Value('DOUBLE_SUMMARY_METRIC_TYPE'), metrics.DoubleSummaryMetric),
            (mp.MetricType.Value('SCALAR_METRIC_TYPE'), metrics.ScalarMetric),
            (mp.MetricType.Value('PLOTLY_METRIC_TYPE'), metrics.PlotlyMetric),
            (mp.MetricType.Value('IMAGE_METRIC_TYPE'), metrics.ImageMetric)]

        for metric_type, metric_class in metric_type_list:
            modified_msg = copy.copy(msg)
            modified_msg.type = metric_type
            unpacked = metrics.Metric.unpack_common_fields(modified_msg)
            self.assertEqual(type(unpacked), metric_class)

        INVALID_TYPE = -1
        for metric_type in (
                mp.MetricType.Value('NO_METRIC_TYPE'),
                INVALID_TYPE):
            with self.assertRaises(ValueError):
                modified_msg = copy.copy(msg)
                modified_msg.type = metric_type
                msg = metrics.Metric.unpack_common_fields(modified_msg)

    def test_scalar_metric(self) -> None:
        # CONSTRUCTION
        job_id = uuid.uuid4()
        metric = metrics.ScalarMetric(
            "test_metric",
            "A metric for testing",
            MetricStatus.PASSED_METRIC_STATUS,
            MetricImportance.ZERO_IMPORTANCE,
            should_display=True,
            blocking=False,
            parent_job_id=job_id,
            order=None,
            value=24.0,
            failure_definition=None,
            unit="")

        # SETTING
        test_value = 3
        self.assertIs(metric, metric.with_value(test_value))
        self.assertEqual(metric.value, test_value)

        test_unit = 'm'
        self.assertIs(metric, metric.with_unit(test_unit))
        self.assertEqual(metric.unit, test_unit)

        test_failure_definition = metrics_utils.DoubleFailureDefinition(
            fails_above=1.0,
            fails_below=0.0,
        )
        self.assertIs(
            metric,
            metric.with_failure_definition(test_failure_definition))
        self.assertEqual(metric.failure_definition, test_failure_definition)

        # PACKING
        msg = metric.pack()
        self.assertEqual(msg.type, mp.MetricType.Value("SCALAR_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField('scalar_metric_values'))
        values = msg.metric_values.scalar_metric_values
        self.assertEqual(values.value, metric.value)
        assert metric.failure_definition is not None
        self.assertEqual(
            values.failure_definition,
            metric.failure_definition.pack())
        self.assertEqual(values.unit, metric.unit)

        optional_attr_list = ["value", "failure_definition", "unit"]
        for attr in optional_attr_list:
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            values = modified_msg.metric_values.scalar_metric_values
            self.assertFalse(values.HasField(attr))

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)

    def test_double_over_time_metric(self) -> None:
        job_id = uuid.uuid4()

        time_data = metrics.SeriesMetricsData(
            name="times",
            series=np.array([metrics_utils.Timestamp(secs=0, nanos=0),
                             metrics_utils.Timestamp(secs=5, nanos=0)]),
            unit='m')
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5, 0.6]),
            unit='m',
            index_data=time_data)
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='',
            index_data=time_data)
        failure_definition = metrics_utils.DoubleFailureDefinition(
            fails_above=1.0,
            fails_below=0.0,
        )
        series_name = "test data"

        metric = metrics.DoubleOverTimeMetric(
            name='test metric',
            description='a test double over time metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            start_time=metrics_utils.Timestamp(secs=0, nanos=0),
            end_time=metrics_utils.Timestamp(secs=5, nanos=0),
            y_axis_name="my y axis",
        )

        self.assertEqual(metric.doubles_over_time_data, [])
        self.assertEqual(metric.statuses_over_time_data, [])
        self.assertEqual(metric.failure_definitions, [])
        self.assertEqual(metric.legend_series_names, [])

        self.assertEqual(
            metric, metric.append_doubles_over_time_data(
                value_data, series_name))
        self.assertEqual(metric.doubles_over_time_data, [value_data])
        self.assertEqual(metric.legend_series_names, [series_name])

        self.assertEqual(
            metric, metric.append_statuses_over_time_data(status_data))
        self.assertEqual(metric.statuses_over_time_data, [status_data])

        self.assertEqual(
            metric, metric.with_failure_definitions(
                [failure_definition]))
        self.assertEqual(metric.failure_definitions, [failure_definition])

        self.assertEqual(metric, metric.with_doubles_over_time_data([]))
        self.assertEqual(metric.doubles_over_time_data, [])
        self.assertEqual(metric, metric.with_statuses_over_time_data([]))
        self.assertEqual(metric.statuses_over_time_data, [])
        self.assertEqual(metric, metric.with_legend_series_names([]))
        self.assertEqual(metric.legend_series_names, [])

        new_start_time = metrics_utils.Timestamp(secs=1, nanos=0)
        self.assertEqual(metric, metric.with_start_time(new_start_time))
        self.assertEqual(metric.start_time, new_start_time)

        new_end_time = metrics_utils.Timestamp(secs=4, nanos=0)
        self.assertEqual(metric, metric.with_end_time(new_end_time))
        self.assertEqual(metric.end_time, new_end_time)

        new_y_axis_name = "my other y axis"
        self.assertEqual(metric, metric.with_y_axis_name(new_y_axis_name))
        self.assertEqual(metric.y_axis_name, new_y_axis_name)

    def test_double_over_time_metric_pack(self) -> None:
        job_id = uuid.uuid4()

        time_data = metrics.SeriesMetricsData(
            name="times",
            series=np.array([metrics_utils.Timestamp(secs=0, nanos=0),
                             metrics_utils.Timestamp(secs=5, nanos=0)]),
            unit='m')
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5, 0.6]),
            unit='m',
            index_data=time_data)
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='',
            index_data=time_data)
        failure_definition = metrics_utils.DoubleFailureDefinition(
            fails_above=1.0,
            fails_below=0.0,
        )
        series_name = "test data"

        # Use the constructor to initialize the data this time, in contrast with
        # the above test.
        metric = metrics.DoubleOverTimeMetric(
            name='test metric',
            description='a test double over time metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            doubles_over_time_data=[value_data],
            statuses_over_time_data=[status_data],
            failure_definitions=[failure_definition],
            start_time=metrics_utils.Timestamp(secs=0, nanos=0),
            end_time=metrics_utils.Timestamp(secs=5, nanos=0),
            y_axis_name="my y axis",
            legend_series_names=[series_name],
        )

        msg = metric.pack()

        self.assert_common_fields_match(msg=msg, metric=metric)
        self.assertEqual(msg.type, mp.MetricType.Value(
            "DOUBLE_OVER_TIME_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField(
            "double_over_time_metric_values"))
        values = msg.metric_values.double_over_time_metric_values
        self.assertEqual(len(values.doubles_over_time_data_id), 1)
        self.assertEqual(
            values.doubles_over_time_data_id[0].id,
            metrics_utils.pack_uuid_to_proto(
                value_data.id))
        self.assertEqual(len(values.statuses_over_time_data_id), 1)
        self.assertEqual(
            values.statuses_over_time_data_id[0].id,
            metrics_utils.pack_uuid_to_proto(
                status_data.id))
        self.assertEqual(len(values.legend_series_names), 1)
        self.assertEqual(values.legend_series_names[0], series_name)
        assert metric.start_time is not None
        self.assertEqual(values.start_time, metric.start_time.pack())
        assert metric.end_time is not None
        self.assertEqual(values.end_time, metric.end_time.pack())
        self.assertEqual(values.y_axis_name, metric.y_axis_name)
        self.assertEqual(values.y_axis_name, metric.y_axis_name)
        self.assertIn(failure_definition.pack(), values.failure_definition)

        # Now set things to None:
        def get_values(msg: Any) -> Any:
            return msg.metric_values.double_over_time_metric_values

        for attr in ('start_time', 'end_time', 'y_axis_name'):
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            self.assertFalse(get_values(modified_msg).HasField(attr))

        modified_metric = copy.copy(metric)
        modified_metric.with_legend_series_names([None])
        modified_msg = modified_metric.pack()
        values = get_values(modified_msg)
        self.assertEqual(set(values.legend_series_names), {value_data.name})

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)
        ids = [uuid.UUID(data.metrics_data_id.id.data)
               for data in output.metrics_msg.metrics_data]
        self.assertIn(value_data.id, ids)
        self.assertIn(status_data.id, ids)
        self.assertIn(time_data.id, ids)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)

    def test_states_over_time_metric(self) -> None:
        job_id = uuid.uuid4()

        time_data = metrics.SeriesMetricsData(
            name="times",
            series=np.array([metrics_utils.Timestamp(secs=0, nanos=0),
                             metrics_utils.Timestamp(secs=5, nanos=0)]),
            unit='m')
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array(["good state", "good state"]),
            unit='',
            index_data=time_data)
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='',
            index_data=time_data)
        states_set = {"good state", "bad state"}
        failure_states = {"bad state"}
        series_name = "test data"

        metric = metrics.StatesOverTimeMetric(
            name='test metric',
            description='a test states over time metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
        )

        self.assertEqual(metric.states_over_time_data, [])
        self.assertEqual(metric.statuses_over_time_data, [])
        self.assertEqual(metric.legend_series_names, [])

        self.assertEqual(
            metric, metric.append_states_over_time_data(
                value_data, series_name))
        self.assertEqual(metric.states_over_time_data, [value_data])
        self.assertEqual(metric.legend_series_names, [series_name])

        self.assertEqual(
            metric, metric.append_statuses_over_time_data(status_data))
        self.assertEqual(metric.statuses_over_time_data, [status_data])

        self.assertEqual(metric, metric.with_states_over_time_data([]))
        self.assertEqual(metric.states_over_time_data, [])
        self.assertEqual(metric, metric.with_statuses_over_time_data([]))
        self.assertEqual(metric.statuses_over_time_data, [])
        self.assertEqual(metric, metric.with_legend_series_names([]))
        self.assertEqual(metric.legend_series_names, [])

        self.assertEqual(metric, metric.with_states_set(states_set))
        self.assertEqual(metric.states_set, states_set)

        self.assertEqual(metric, metric.with_failure_states(failure_states))
        self.assertEqual(metric.failure_states, failure_states)

    def test_states_over_time_metric_pack(self) -> None:
        job_id = uuid.uuid4()

        time_data = metrics.SeriesMetricsData(
            name="times",
            series=np.array([metrics_utils.Timestamp(secs=0, nanos=0),
                             metrics_utils.Timestamp(secs=5, nanos=0)]),
            unit='m')
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array(["good state", "good state"]),
            unit='',
            index_data=time_data)
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='',
            index_data=time_data)
        states_set = {"good state", "bad state"}
        failure_states = {"bad state"}
        series_name = "test data"

        # Use the constructor to initialize the data this time, in contrast with
        # the above test.
        metric = metrics.StatesOverTimeMetric(
            name='test metric',
            description='a test states over time metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            states_over_time_data=[value_data],
            statuses_over_time_data=[status_data],
            states_set=states_set,
            failure_states=failure_states,
            legend_series_names=[series_name],
        )

        msg = metric.pack()

        self.assert_common_fields_match(msg=msg, metric=metric)
        self.assertEqual(msg.type, mp.MetricType.Value(
            "STATES_OVER_TIME_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField(
            "states_over_time_metric_values"))
        values = msg.metric_values.states_over_time_metric_values
        self.assertEqual(len(values.states_over_time_data_id), 1)
        self.assertEqual(
            values.states_over_time_data_id[0].id,
            metrics_utils.pack_uuid_to_proto(
                value_data.id))
        self.assertEqual(len(values.statuses_over_time_data_id), 1)
        self.assertEqual(
            values.statuses_over_time_data_id[0].id,
            metrics_utils.pack_uuid_to_proto(
                status_data.id))
        self.assertEqual(len(values.legend_series_names), 1)
        self.assertEqual(values.legend_series_names[0], series_name)
        self.assertEqual(set(values.states_set), metric.states_set)
        self.assertEqual(set(values.failure_states), metric.failure_states)

        # Now set things to None:
        def get_values(msg: Any) -> Any:
            return msg.metric_values.states_over_time_metric_values

        for attr in ('states_set', 'failure_states'):
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            self.assertEqual(len(getattr(get_values(modified_msg), attr)), 0)

        modified_metric = copy.copy(metric)
        modified_metric.with_legend_series_names([None])
        modified_msg = modified_metric.pack()
        values = get_values(modified_msg)
        self.assertEqual(len(values.legend_series_names), 1)
        self.assertEqual(values.legend_series_names[0], value_data.name)

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)
        ids = [uuid.UUID(data.metrics_data_id.id.data)
               for data in output.metrics_msg.metrics_data]
        self.assertIn(value_data.id, ids)
        self.assertIn(status_data.id, ids)
        self.assertIn(time_data.id, ids)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)

    def test_states_over_time_with_states_over_time_series(self) -> None:
        """A special unit test for StatesOverTimeMetric.with_states_over_time_series()"""

        value_data = np.array(['good', 'bad'])
        time_data = np.array([metrics_utils.Timestamp(secs=0, nanos=0),
                             metrics_utils.Timestamp(secs=5, nanos=0)])

        metric = metrics.StatesOverTimeMetric(
            name='test metric',
        )
        metric.with_states_over_time_series(
            states_over_time_series={'a_states': value_data},
            units={'a_states': 'dabloons'},
            indices={'a_states': time_data},
            legend_series_names={'a_states': 'My A States'})

        self.assertEqual(len(metric.states_over_time_data), 1)

        data = metric.states_over_time_data[0]
        assert isinstance(data, metrics.SeriesMetricsData)
        self.assertEqual(data.name, 'a_states')
        self.assertTrue((data.series == value_data).all())
        self.assertEqual(data.unit, 'dabloons')
        assert isinstance(data.index_data, metrics.SeriesMetricsData)
        self.assertTrue((data.index_data.series == time_data).all())

        metric = metrics.StatesOverTimeMetric(
            name='test metric',
        )
        metric.with_states_over_time_series(
            states_over_time_series={'a_states': value_data})
        self.assertEqual(len(metric.states_over_time_data), 1)
        data = metric.states_over_time_data[0]
        self.assertEqual(data.name, 'a_states')
        self.assertIs(data.unit, None)
        self.assertEqual(metric.legend_series_names, [None])

    def test_line_plot_metric(self) -> None:
        job_id = uuid.uuid4()

        x_value_data = metrics.SeriesMetricsData(
            name="x values",
            series=np.array([0.5, 0.6]),
            unit='m')

        y_value_data = metrics.SeriesMetricsData(
            name="y values",
            series=np.array([0.1, 0.2]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='')

        series_name = "test data"

        metric = metrics.LinePlotMetric(
            name='test metric',
            description='a test line plot  metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
        )

        self.assertEqual(metric.x_doubles_data, [])
        self.assertEqual(metric.y_doubles_data, [])
        self.assertEqual(metric.statuses_data, [])
        self.assertEqual(metric.legend_series_names, [])

        self.assertEqual(
            metric, metric.append_series_data(
                x_value_data,
                y_value_data,
                series_name,
            ))
        self.assertEqual(metric.x_doubles_data, [x_value_data])
        self.assertEqual(metric.y_doubles_data, [y_value_data])
        self.assertEqual(metric.legend_series_names, [series_name])

        self.assertEqual(
            metric, metric.append_statuses_data(status_data))
        self.assertEqual(metric.statuses_data, [status_data])

        self.assertEqual(metric, metric.with_legend_series_names([]))
        self.assertEqual(metric.legend_series_names, [])

        x_axis_name = "my x axis name"
        self.assertEqual(metric, metric.with_x_axis_name(x_axis_name))
        self.assertEqual(metric.x_axis_name, x_axis_name)

        y_axis_name = "my x axis name"
        self.assertEqual(metric, metric.with_y_axis_name(y_axis_name))
        self.assertEqual(metric.y_axis_name, y_axis_name)

    def test_line_plot_metric_pack(self) -> None:
        job_id = uuid.uuid4()

        x_value_data = metrics.SeriesMetricsData(
            name="x values",
            series=np.array([0.5, 0.6]),
            unit='m')

        y_value_data = metrics.SeriesMetricsData(
            name="y values",
            series=np.array([0.1, 0.2]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='')

        series_name = "test data"

        # Use the constructor to initialize the data this time, in contrast with
        # the above test.
        metric = metrics.LinePlotMetric(
            name='test metric',
            description='a test line plot  metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            x_doubles_data=[x_value_data],
            y_doubles_data=[y_value_data],
            statuses_data=[status_data],
            x_axis_name="my x axis name",
            y_axis_name="my y axis name",
            legend_series_names=[series_name],
        )
        msg = metric.pack()
        self.assert_common_fields_match(msg=msg, metric=metric)
        self.assertEqual(msg.type, mp.MetricType.Value(
            "LINE_PLOT_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField(
            "line_plot_metric_values"))
        values = msg.metric_values.line_plot_metric_values
        self.assertEqual({data_id.id.data for data_id in values.x_doubles_data_id},
                         {str(x_value_data.id)})
        self.assertEqual({data_id.id.data for data_id in values.y_doubles_data_id},
                         {str(y_value_data.id)})
        self.assertEqual({data_id.id.data for data_id in values.statuses_data_id},
                         {str(status_data.id)})
        self.assertEqual(set(values.legend_series_names),
                         {series_name})
        for attr in ("x_axis_name", "y_axis_name"):
            self.assertEqual(getattr(values, attr), getattr(metric, attr))

        # Now set things to None:
        def get_values(msg: Any) -> Any:
            return msg.metric_values.line_plot_metric_values

        for attr in ('x_axis_name', 'y_axis_name'):
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            self.assertFalse(get_values(modified_msg).HasField(attr))

        modified_metric = copy.copy(metric)
        modified_metric.with_legend_series_names([None])
        modified_msg = modified_metric.pack()
        values = get_values(modified_msg)
        self.assertEqual(len(values.legend_series_names), 1)
        self.assertEqual(values.legend_series_names[0], y_value_data.name)

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)
        ids = [uuid.UUID(data.metrics_data_id.id.data)
               for data in output.metrics_msg.metrics_data]
        self.assertIn(x_value_data.id, ids)
        self.assertIn(y_value_data.id, ids)
        self.assertIn(status_data.id, ids)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)

    def test_bar_chart_metric(self) -> None:
        job_id = uuid.uuid4()
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5, 0.6]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='')
        series_name = "test data"

        metric = metrics.BarChartMetric(
            name='test metric',
            description='a test bar chart metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
        )

        self.assertEqual(metric.values_data, [])
        self.assertEqual(metric.statuses_data, [])
        self.assertEqual(metric.legend_series_names, [])

        self.assertEqual(
            metric, metric.append_values_data(
                value_data, series_name))
        self.assertEqual(metric.values_data, [value_data])
        self.assertEqual(metric.legend_series_names, [series_name])

        self.assertEqual(
            metric, metric.append_statuses_data(status_data))
        self.assertEqual(metric.statuses_data, [status_data])

        self.assertEqual(metric, metric.with_legend_series_names([]))
        self.assertEqual(metric.legend_series_names, [])

        new_x_axis_name = "my x axis"
        self.assertEqual(metric, metric.with_x_axis_name(new_x_axis_name))
        self.assertEqual(metric.x_axis_name, new_x_axis_name)

        new_y_axis_name = "my y axis"
        self.assertEqual(metric, metric.with_y_axis_name(new_y_axis_name))
        self.assertEqual(metric.y_axis_name, new_y_axis_name)

        new_stack_bars = True
        self.assertEqual(metric, metric.with_stack_bars(new_stack_bars))
        self.assertEqual(metric.stack_bars, new_stack_bars)

    def test_bar_chart_metric_pack(self) -> None:
        job_id = uuid.uuid4()
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5, 0.6]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='')
        series_name = "test data"

        # Use the constructor to initialize the data this time, in contrast with
        # the above test.
        metric = metrics.BarChartMetric(
            name='test metric',
            description='a test bar chart metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            values_data=[value_data],
            statuses_data=[status_data],
            legend_series_names=[series_name],
            x_axis_name="my x axis",
            y_axis_name="my y axis",
            stack_bars=True,
        )
        msg = metric.pack()

        self.assert_common_fields_match(msg=msg, metric=metric)
        self.assertEqual(msg.type, mp.MetricType.Value(
            "BAR_CHART_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField(
            "bar_chart_metric_values"))
        values = msg.metric_values.bar_chart_metric_values
        self.assertEqual({data_id.id.data for data_id in values.values_data_id},
                         {str(value_data.id)})
        self.assertEqual({data_id.id.data for data_id in values.statuses_data_id},
                         {str(status_data.id)})
        self.assertEqual(set(values.legend_series_names),
                         {series_name})

        for attr in ("x_axis_name", "y_axis_name", "stack_bars"):
            self.assertEqual(getattr(values, attr), getattr(metric, attr))

        # Now set things to None:
        def get_values(msg: Any) -> Any:
            return msg.metric_values.bar_chart_metric_values

        for attr in ('x_axis_name', 'y_axis_name', 'stack_bars'):
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            self.assertFalse(get_values(modified_msg).HasField(attr))

        modified_metric = copy.copy(metric)
        modified_metric.with_legend_series_names([None])
        modified_msg = modified_metric.pack()
        values = get_values(modified_msg)
        self.assertEqual(set(values.legend_series_names), {value_data.name})

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)
        ids = [uuid.UUID(data.metrics_data_id.id.data)
               for data in output.metrics_msg.metrics_data]
        self.assertIn(value_data.id, ids)
        self.assertIn(status_data.id, ids)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)

    def test_histogram_metric(self) -> None:
        job_id = uuid.uuid4()
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5, 0.6]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='')

        buckets = [metrics_utils.HistogramBucket(lower=0.0,
                                                 upper=0.5),
                   metrics_utils.HistogramBucket(lower=0.5,
                                                 upper=1.0)]

        metric = metrics.HistogramMetric(
            name='test metric',
            description='a test histogram metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
        )

        self.assertIs(metric.values_data, None)
        self.assertIs(metric.statuses_data, None)
        self.assertIs(metric.buckets, None)
        self.assertIs(metric.lower_bound, None)
        self.assertIs(metric.upper_bound, None)
        self.assertIs(metric.x_axis_name, None)

        self.assertEqual(
            metric, metric.with_values_data(value_data))
        self.assertEqual(metric.values_data, value_data)

        self.assertEqual(
            metric, metric.with_statuses_data(status_data))
        self.assertEqual(metric.statuses_data, status_data)

        self.assertEqual(
            metric, metric.with_buckets(buckets))
        self.assertEqual(metric.buckets, buckets)

        self.assertEqual(
            metric, metric.with_lower_bound(buckets[0].lower))
        self.assertEqual(metric.lower_bound, buckets[0].lower)

        self.assertEqual(
            metric, metric.with_upper_bound(buckets[-1].upper))
        self.assertEqual(metric.upper_bound, buckets[-1].upper)

        new_x_axis_name = "my x axis"
        self.assertEqual(metric, metric.with_x_axis_name(new_x_axis_name))
        self.assertEqual(metric.x_axis_name, new_x_axis_name)

    def test_histogram_metric_pack(self) -> None:
        job_id = uuid.uuid4()
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5, 0.6]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array(2 * [MetricStatus.PASSED_METRIC_STATUS]),
            unit='')

        buckets = [metrics_utils.HistogramBucket(lower=0.0,
                                                 upper=0.5),
                   metrics_utils.HistogramBucket(lower=0.5,
                                                 upper=1.0)]

        metric = metrics.HistogramMetric(
            name='test metric',
            description='a test histogram metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            values_data=value_data,
            statuses_data=status_data,
            buckets=buckets,
            lower_bound=buckets[0].lower,
            upper_bound=buckets[-1].upper,
            x_axis_name="my x axis",
        )

        msg = metric.pack()

        self.assert_common_fields_match(msg=msg, metric=metric)
        self.assertEqual(msg.type, mp.MetricType.Value(
            "HISTOGRAM_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField(
            "histogram_metric_values"))
        values = msg.metric_values.histogram_metric_values
        self.assertEqual(values.values_data_id.id,
                         metrics_utils.pack_uuid_to_proto(value_data.id))
        self.assertEqual(values.statuses_data_id.id,
                         metrics_utils.pack_uuid_to_proto(status_data.id))
        self.assertEqual(values.lower_bound, metric.lower_bound)
        self.assertEqual(values.upper_bound, metric.upper_bound)
        self.assertEqual(values.x_axis_name, metric.x_axis_name)

        assert metric.buckets is not None
        for bucket_proto, bucket in zip(values.buckets, metric.buckets):
            self.assertEqual(bucket_proto, bucket.pack())

        # Now set things to None:
        def get_values(msg: Any) -> Any:
            return msg.metric_values.histogram_metric_values

        for attr in ('values_data', 'statuses_data'):
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            self.assertFalse(get_values(modified_msg).HasField(attr + "_id"))

        for attr in ('x_axis_name', 'lower_bound', 'upper_bound'):
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            self.assertFalse(get_values(modified_msg).HasField(attr))

        modified_metric = copy.copy(metric)
        modified_metric.buckets = None
        modified_msg = modified_metric.pack()
        self.assertEqual(len(get_values(modified_msg).buckets), 0)

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)
        ids = [uuid.UUID(data.metrics_data_id.id.data)
               for data in output.metrics_msg.metrics_data]
        self.assertIn(value_data.id, ids)
        self.assertIn(status_data.id, ids)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)

        # Check empty series, mainly for test coverage
        metric = metrics.HistogramMetric(name="some empty metric")
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 2)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)

    def test_double_summary_metric(self) -> None:
        job_id = uuid.uuid4()
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array([MetricStatus.PASSED_METRIC_STATUS]),
            unit='')
        index = 0
        failure_definition = metrics_utils.DoubleFailureDefinition(
            fails_above=1.0,
            fails_below=0.0,
        )
        metric = metrics.DoubleSummaryMetric(
            name='test metric',
            description='a test histogram metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
        )

        self.assertIs(metric.value_data, None)
        self.assertIs(metric.status_data, None)
        self.assertIs(metric.index, None)
        self.assertIs(metric.failure_definition, None)

        self.assertEqual(
            metric, metric.with_value_data(value_data))
        self.assertEqual(metric.value_data, value_data)

        self.assertEqual(
            metric, metric.with_status_data(status_data))
        self.assertEqual(metric.status_data, status_data)

        self.assertEqual(
            metric, metric.with_index(index))
        self.assertEqual(metric.index, index)

        self.assertEqual(
            metric, metric.with_failure_definition(failure_definition))
        self.assertEqual(metric.failure_definition, failure_definition)

    def test_double_summary_metric_pack(self) -> None:
        job_id = uuid.uuid4()
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array([MetricStatus.PASSED_METRIC_STATUS]),
            unit='')
        index = 0
        failure_definition = metrics_utils.DoubleFailureDefinition(
            fails_above=1.0,
            fails_below=0.0,
        )
        metric = metrics.DoubleSummaryMetric(
            name='test metric',
            description='a test histogram metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            value_data=value_data,
            status_data=status_data,
            index=index,
            failure_definition=failure_definition,
        )

        msg = metric.pack()

        self.assert_common_fields_match(msg=msg, metric=metric)
        self.assertEqual(msg.type, mp.MetricType.Value(
            "DOUBLE_SUMMARY_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField(
            "double_metric_values"))
        values = msg.metric_values.double_metric_values
        self.assertEqual(values.value_data_id.id,
                         metrics_utils.pack_uuid_to_proto(value_data.id))
        self.assertEqual(values.status_data_id.id,
                         metrics_utils.pack_uuid_to_proto(status_data.id))
        self.assertTrue(values.HasField('series_index'))
        self.assertEqual(values.series_index, metric.index)
        assert metric.failure_definition is not None
        self.assertEqual(
            values.failure_definition,
            metric.failure_definition.pack())

        # Now set things to None:
        def get_values(msg: Any) -> Any:
            return msg.metric_values.double_metric_values

        for attr in ('value_data', 'status_data'):
            modified_metric = copy.copy(metric)
            setattr(modified_metric, attr, None)
            modified_msg = modified_metric.pack()
            self.assertFalse(get_values(modified_msg).HasField(attr + "_id"))

        modified_metric = copy.copy(metric)
        modified_metric.index = None
        modified_msg = modified_metric.pack()
        self.assertIs(get_values(modified_msg).WhichOneof('index'), None)

        modified_metric = copy.copy(metric)
        modified_metric.index = 'string key'
        modified_msg = modified_metric.pack()
        self.assertEqual(get_values(modified_msg).string_index, 'string key')

        uuid_key = uuid.uuid4()
        modified_metric = copy.copy(metric)
        modified_metric.index = uuid_key
        modified_msg = modified_metric.pack()
        self.assertEqual(
            get_values(modified_msg).uuid_index,
            metrics_utils.pack_uuid_to_proto(uuid_key))

        time_key = metrics_utils.Timestamp(secs=1, nanos=3)
        modified_metric = copy.copy(metric)
        modified_metric.index = time_key
        modified_msg = modified_metric.pack()
        self.assertEqual(
            get_values(modified_msg).timestamp_index,
            time_key.pack())

        bad_type_key = cast(None, metrics.SeriesMetricsData(name="whoops"))
        modified_metric = copy.copy(metric)
        modified_metric.index = bad_type_key
        with self.assertRaises(ValueError):
            modified_msg = modified_metric.pack()

        modified_metric = copy.copy(metric)
        modified_metric.failure_definition = None
        modified_msg = modified_metric.pack()
        self.assertFalse(
            get_values(modified_msg).HasField('failure_definition'))

    def test_double_summary_metric_recursive_pack(self) -> None:
        job_id = uuid.uuid4()
        value_data = metrics.SeriesMetricsData(
            name="values",
            series=np.array([0.5]),
            unit='m')
        status_data = metrics.SeriesMetricsData(
            name="statuses",
            series=np.array([MetricStatus.PASSED_METRIC_STATUS]),
            unit='')
        index = 0
        failure_definition = metrics_utils.DoubleFailureDefinition(
            fails_above=1.0,
            fails_below=0.0,
        )
        metric = metrics.DoubleSummaryMetric(
            name='test metric',
            description='a test histogram metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            value_data=value_data,
            status_data=status_data,
            index=index,
            failure_definition=failure_definition,
        )
        msg = metric.pack()

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)
        ids = [uuid.UUID(data.metrics_data_id.id.data)
               for data in output.metrics_msg.metrics_data]
        self.assertIn(value_data.id, ids)
        self.assertIn(status_data.id, ids)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)

        # Check empty series, mainly for test coverage
        metric = metrics.DoubleSummaryMetric(name="some empty metric")
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 2)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)

    def test_plotly_metric(self) -> None:
        # CONSTRUCTION
        job_id = uuid.uuid4()
        metric = metrics.PlotlyMetric(
            "test_metric",
            "A metric for testing",
            MetricStatus.PASSED_METRIC_STATUS,
            MetricImportance.ZERO_IMPORTANCE,
            should_display=True,
            blocking=False,
            parent_job_id=job_id,
            order=None,
            plotly_data=None,
        )

        # SETTING
        test_data = Struct()
        test_data["test"] = "test"
        self.assertIs(metric, metric.with_plotly_data(test_data))
        self.assertEqual(metric.plotly_data, test_data)

        # PACKING
        msg = metric.pack()
        self.assertEqual(msg.type, mp.MetricType.Value("PLOTLY_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField('plotly_metric_values'))
        values = msg.metric_values.plotly_metric_values
        self.assertEqual(values.json, metric.plotly_data)

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)

    def test_image_metric(self) -> None:
        job_id = uuid.uuid4()

        file_data = metrics.ExternalFileMetricsData(
            name="an external image",
            filename='test.gif')

        metric = metrics.ImageMetric(
            name='test metric',
            description='a test image metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
        )

        self.assertEqual(metric.image_data, None)

        self.assertEqual(
            metric, metric.with_image_data(
                file_data))

    def test_image_metric_pack(self) -> None:
        job_id = uuid.uuid4()

        image_data = metrics.ExternalFileMetricsData(
            name="an external image",
            filename='test.gif')

        # Use the constructor to initialize the data this time, in contrast with
        # the above test.
        metric = metrics.ImageMetric(
            name='test metric',
            description='a test image metric',
            status=MetricStatus.PASSED_METRIC_STATUS,
            importance=MetricImportance.ZERO_IMPORTANCE,
            blocking=False,
            should_display=True,
            parent_job_id=job_id,
            order=0.5,
            image_data=image_data
        )

        msg = metric.pack()

        self.assert_common_fields_match(msg=msg, metric=metric)
        self.assertEqual(msg.type, mp.MetricType.Value(
            "IMAGE_METRIC_TYPE"))
        self.assertTrue(msg.metric_values.HasField(
            "image_metric_values"))
        values = msg.metric_values.image_metric_values
        self.assertEqual(
            values.image_data_id.id,
            metrics_utils.pack_uuid_to_proto(
                image_data.id))

        output = metrics_utils.ResimMetricsOutput()
        metric.recursively_pack_into(output)
        self.assertIn(metric.id, output.packed_ids)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 1)
        ids = [uuid.UUID(data.metrics_data_id.id.data)
                for data in output.metrics_msg.metrics_data]
        self.assertIn(image_data.id, ids)
        self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # Check no duplication
        metric.recursively_pack_into(output)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 1)

    def test_metrics_data(self) -> None:
        index_data = metrics.SeriesMetricsData(
            name="index data",
            series=np.array([1., 2., 3.]),
        )
        metrics_data = metrics.SeriesMetricsData(
            name="metrics data",
            series=np.array([1., 2., 3.]),
        )
        unit = 'm'

        self.assertEqual(metrics_data, metrics_data.with_unit(unit))
        self.assertEqual(metrics_data.unit, unit)

        self.assertEqual(
            metrics_data,
            metrics_data.with_index_data(index_data))
        self.assertEqual(metrics_data.index_data, index_data)

        self.assertEqual(metrics_data, metrics_data)
        self.assertNotEqual(metrics_data, 4)
        self.assertNotEqual(metrics_data, index_data)

        with self.assertRaises(NotImplementedError):
            metrics.MetricsData.map(metrics_data, lambda *_: _, '')
        with self.assertRaises(NotImplementedError):
            metrics.MetricsData.group_by(metrics_data, index_data)
        with self.assertRaises(NotImplementedError):
            metrics.MetricsData.pack(metrics_data)

        output = metrics_utils.ResimMetricsOutput()
        metrics_data.recursively_pack_into(output)
        self.assertEqual(output.packed_ids, {index_data.id, metrics_data.id})
        self.assertEqual({uuid.UUID(md.metrics_data_id.id.data)
                         for md in output.metrics_msg.metrics_data},
                         {index_data.id, metrics_data.id})
        metrics_data.recursively_pack_into(output)
        self.assertEqual(output.packed_ids, {index_data.id, metrics_data.id})
        self.assertEqual({uuid.UUID(md.metrics_data_id.id.data)
                         for md in output.metrics_msg.metrics_data},
                         {index_data.id, metrics_data.id})

    def test_series_metrics_data(self) -> None:
        index_data = metrics.SeriesMetricsData(
            name="index data",
            series=np.array([1., 2., 3.]),
        )
        series = np.array([4., 5., 6.])
        metrics_data = metrics.SeriesMetricsData(
            name="metrics data",
            unit='m',
            index_data=index_data,
        )
        self.assertEqual(metrics_data, metrics_data.with_series(series))
        self.assertTrue((metrics_data.series == series).all())

        negated_metrics_data = metrics_data.map(lambda arr, i: -arr[i],
                                                'negated metrics data',
                                                'm')

        for normal, negated in zip(metrics_data.series,
                                   negated_metrics_data.series):
            self.assertEqual(normal, -negated)

        grouping_data = metrics.SeriesMetricsData(
            name="grouping data",
            series=np.array(["yes", "no", "yes"])
        )

        grouped_data = metrics_data.group_by(grouping_data,
                                             'grouped data',
                                             'grouped data index')

        expected = {"yes": np.array([4., 6.]),
                    "no": np.array([5.])}
        expected_index = {"yes": np.array([1., 3.]),
                          "no": np.array([2.])}

        assert grouped_data is not None
        assert grouped_data.index_data is not None
        for key, val in expected.items():
            self.assertTrue(
                (val == grouped_data.category_to_series[key]).all())
            self.assertTrue(
                (expected_index[key] == grouped_data.index_data.category_to_series[key]).all())

        self.assertEqual(grouped_data.name, 'grouped data')
        self.assertEqual(grouped_data.index_data.name, 'grouped data index')

        # Cover the override case:
        regrouped_index = index_data.group_by(grouping_data,
                                              'regrouped data',
                                              'regrouped data index',
                                              grouped_data.index_data)

        for key, val in expected_index.items():
            self.assertTrue(
                (val == regrouped_index.category_to_series[key]).all())
        self.assertEqual(regrouped_index.index_data, grouped_data.index_data)

        assert regrouped_index is not None
        assert regrouped_index.index_data is not None

        self.assertEqual(regrouped_index.name, 'regrouped data')
        self.assertEqual(regrouped_index.index_data.name, 'grouped data index')

        # Check the auto naming:
        autonamed_grouped_data = metrics_data.group_by(grouping_data)
        assert autonamed_grouped_data is not None
        assert autonamed_grouped_data.index_data is not None

        self.assertEqual(
            autonamed_grouped_data.name,
            "metrics data-grouped-by-grouping data")
        self.assertEqual(
            autonamed_grouped_data.index_data.name,
            "index data-grouped-by-grouping data")

        autonamed_grouped_index = index_data.group_by(grouping_data)
        assert autonamed_grouped_index is not None
        assert autonamed_grouped_index.index_data is None

        self.assertEqual(
            autonamed_grouped_index.name,
            "index data-grouped-by-grouping data")

    def test_series_metrics_data_pack(self) -> None:
        index_data = metrics.SeriesMetricsData(
            name="index data",
            series=np.array([1., 2., 3.]),
        )
        series = np.array([4., 5., 6.])
        metrics_data = metrics.SeriesMetricsData(
            name="metrics data",
            series=series,
            unit='m',
            index_data=index_data,
        )

        msg = metrics_data.pack()
        self.assertEqual(
            msg.metrics_data_id.id,
            metrics_utils.pack_uuid_to_proto(
                metrics_data.id))
        self.assertEqual(msg.data_type, mp.MetricsDataType.Value(
            'INDEXED_DOUBLE_SERIES_DATA_TYPE'))

        self.assertEqual(msg.name, metrics_data.name)
        self.assertEqual(msg.unit, metrics_data.unit)
        self.assertFalse(msg.is_per_category)
        self.assertEqual(len(msg.category_names), 0)
        self.assertTrue(msg.is_indexed)
        self.assertEqual(
            msg.index_data_id.id,
            metrics_utils.pack_uuid_to_proto(
                index_data.id))
        self.assertEqual(
            msg.index_data_type,
            mp.MetricsDataType.Value('DOUBLE_SERIES_DATA_TYPE'))
        for packed_val, val in zip(msg.series.doubles.series, series):
            self.assertEqual(packed_val, val)

    def test_grouped_metrics_data(self) -> None:
        category_to_series = {"yes": np.array([4., 6.]),
                              "no": np.array([5.])}

        metrics_data = metrics.GroupedMetricsData(
            name="grouped metrics data",
            unit='m')

        self.assertEqual(
            metrics_data,
            metrics_data.with_category_to_series(category_to_series))

        # Check the case where the category_to_series is passed into the
        # constructor
        other_data = metrics.GroupedMetricsData(
            name="other metrics data",
            category_to_series=category_to_series,
            unit="m",
        )

        for key, val in category_to_series.items():
            self.assertTrue((other_data.category_to_series[key] == val).all())

        negated_data = metrics_data.map(lambda arr, i, cat: -arr[i],
                                        'negated data',
                                        'm')

        self.assertEqual(negated_data.name, 'negated data')
        self.assertEqual(negated_data.unit, 'm')
        for key, val in category_to_series.items():
            self.assertTrue(
                (negated_data.category_to_series[key] == -val).all())

        with self.assertRaises(NotImplementedError):
            metrics_data.group_by(metrics_data)

        new_series = np.array([5.0, 6.0])
        metrics_data.add_category('maybe', new_series)
        self.assertIn('maybe', metrics_data.category_to_series)
        self.assertTrue(
            (metrics_data.category_to_series['maybe'] == new_series).all())

    def test_grouped_metrics_data_pack(self) -> None:
        index_category_to_series = {"yes": np.array([1., 2.]),
                                    "no": np.array([3.])}
        category_to_series = {"yes": np.array([4., 6.]),
                              "no": np.array([5.])}

        index_data = metrics.GroupedMetricsData(
            name="index metrics data",
            category_to_series=index_category_to_series,
        )

        metrics_data = metrics.GroupedMetricsData(
            name="metrics data",
            category_to_series=category_to_series,
            unit='m',
            index_data=index_data,
        )

        msg = metrics_data.pack()
        self.assertEqual(
            msg.metrics_data_id.id,
            metrics_utils.pack_uuid_to_proto(
                metrics_data.id))
        self.assertEqual(msg.data_type, mp.MetricsDataType.Value(
            'INDEXED_DOUBLE_SERIES_DATA_TYPE'))

        self.assertEqual(msg.name, metrics_data.name)
        self.assertEqual(msg.unit, metrics_data.unit)
        self.assertTrue(msg.is_per_category)
        self.assertEqual(set(msg.category_names), {"yes", "no"})
        self.assertTrue(msg.is_indexed)
        self.assertEqual(
            msg.index_data_id.id,
            metrics_utils.pack_uuid_to_proto(
                index_data.id))
        self.assertEqual(
            msg.index_data_type,
            mp.MetricsDataType.Value('DOUBLE_SERIES_DATA_TYPE'))

        for key, packed_series in msg.series_per_category.category_to_series.items():
            for packed_val, val in zip(
                    packed_series.doubles.series, category_to_series[key]):
                self.assertEqual(packed_val, val)

        output = metrics_utils.ResimMetricsOutput()
        metrics_data.recursively_pack_into(output)
        self.assertEqual(output.packed_ids, {index_data.id, metrics_data.id})
        self.assertEqual({uuid.UUID(md.metrics_data_id.id.data)
                         for md in output.metrics_msg.metrics_data},
                         {index_data.id, metrics_data.id})
        metrics_data.recursively_pack_into(output)
        self.assertEqual(output.packed_ids, {index_data.id, metrics_data.id})
        self.assertEqual({uuid.UUID(md.metrics_data_id.id.data)
                         for md in output.metrics_msg.metrics_data},
                         {index_data.id, metrics_data.id})

    def test_external_file_metrics_data(self) -> None:
        filename = "my_file.gif"
        metrics_data = metrics.ExternalFileMetricsData(
            name="file data",
        )
        self.assertEqual(metrics_data, metrics_data.with_filename(filename))
        self.assertTrue(metrics_data.filename == filename)

    def test_external_file_metrics_data_pack(self) -> None:
        filename = "my_file.gif"
        metrics_data = metrics.ExternalFileMetricsData(
            name="metrics data",
            filename=filename,
        )

        msg = metrics_data.pack()
        self.assertEqual(
            msg.metrics_data_id.id,
            metrics_utils.pack_uuid_to_proto(
                metrics_data.id))
        self.assertEqual(msg.data_type, mp.MetricsDataType.Value(
            'EXTERNAL_FILE_DATA_TYPE'))

        self.assertEqual(msg.name, metrics_data.name)
        self.assertEqual(msg.external_file.path, filename)

    def test_event_eq(self) -> None:
        # SETUP
        metric_1 = metrics.ScalarMetric(
            name="test_metric_1",
            value=24.0)
        metric_2 = metrics.ScalarMetric(
            name="test_metric_2",
            value=24.0)

        event = metrics.Event(
            name="my_event",
            description="event description",
            tags=["a tag", "another tag"],
            timestamp=metrics_utils.Timestamp(secs=1, nanos=2),
            importance=MetricImportance.CRITICAL_IMPORTANCE,
            status=MetricStatus.FAIL_BLOCK_METRIC_STATUS,
            metrics=[metric_1, metric_2],
        )

        # Test equality
        self.assertEqual(event, event)

        # Different type
        self.assertNotEqual(event, 3)

        # Non-existent id
        event_with_none_id = copy.copy(event)
        event_with_none_id.id = cast(uuid.UUID, None)
        with self.assertRaises(AssertionError):
            _ = event_with_none_id == event
        with self.assertRaises(AssertionError):
            _ = event == event_with_none_id

        # Different id
        event_with_diff_id = copy.copy(event)
        event_with_diff_id.id = uuid.uuid4()
        self.assertNotEqual(event_with_diff_id, event)
        self.assertNotEqual(event, event_with_diff_id)

    def generate_event_metrics(self) -> list[metrics.Metric]:
        metric_1 = metrics.ScalarMetric(
            name="test_metric_1",
            value=24.0)
        metric_2 = metrics.ScalarMetric(
            name="test_metric_2",
            value=24.0)
        return [metric_1, metric_2]

    def test_event(self) -> None:
        name = "my_event"
        description = "event description"
        tags = ["a tag", "another tag"]
        timestamp = metrics_utils.Timestamp(secs=1, nanos=2)
        importance = MetricImportance.CRITICAL_IMPORTANCE
        status = MetricStatus.FAIL_WARN_METRIC_STATUS
        event_metrics = self.generate_event_metrics()

        event = metrics.Event(
            name=name,
            description=description,
            tags=tags,
            timestamp=timestamp,
            importance=importance,
            status=status,
            metrics=event_metrics,
        )

        self.assertEqual(event, event.with_description(description))
        self.assertEqual(event, event.with_tags(tags))
        self.assertEqual(event, event.with_timestamp(timestamp))
        self.assertEqual(event, event.with_importance(importance))
        self.assertEqual(event, event.with_status(status))
        self.assertEqual(event, event.with_metrics(event_metrics))

        self.assertTrue(event.name == name)
        self.assertTrue(event.description == description)
        self.assertTrue(event.tags == tags)
        self.assertTrue(event.timestamp == timestamp)
        self.assertTrue(event.importance == importance)
        self.assertTrue(event.status == status)
        self.assertTrue(event.metrics == event_metrics)


    def test_event_pack(self) -> None:
        name = "my_event"
        description = "event description"
        tags = ["a tag", "another tag"]
        timestamp = metrics_utils.Timestamp(secs=1, nanos=2)
        importance = MetricImportance.CRITICAL_IMPORTANCE
        status = MetricStatus.FAIL_WARN_METRIC_STATUS
        event_metrics = self.generate_event_metrics()
        event = metrics.Event(
            name=name,
            description=description,
            tags=tags,
            timestamp=timestamp,
            importance=importance,
            status=status,
            metrics=event_metrics,
        )

        msg = event.pack()
        self.assertEqual(
            msg.event_id.id,
            metrics_utils.pack_uuid_to_proto(
                event.id))
        self.assertEqual(msg.name, event.name)
        self.assertTrue(msg.description == event.description)
        self.assertTrue(msg.tags == event.tags)
        assert event.timestamp is not None
        self.assertEqual(msg.timestamp, event.timestamp.pack())
        self.assertTrue(msg.importance == getattr(event.importance, "value"))
        self.assertTrue(msg.status == getattr(event.status, "value"))

        metric_id_uuids = list(map(lambda m: m.id, msg.metrics))
        for metric in event_metrics:
            self.assertIn(metrics_utils.pack_uuid_to_proto(metric.id), metric_id_uuids)

        # output = metrics_utils.ResimMetricsOutput()
        # event.recursively_pack_into(output)
        # self.assertIn(event.id, output.packed_ids)
        # self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        # self.assertEqual(len(output.metrics_msg.metrics_data), 1)
        # ids = [uuid.UUID(data.metrics_data_id.id.data)
        #         for data in output.metrics_msg.metrics_data]
        # self.assertIn(image_data.id, ids)
        # self.assertEqual(output.metrics_msg.job_level_metrics.metrics[0], msg)

        # # Check no duplication
        # metric.recursively_pack_into(output)
        # self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        # self.assertEqual(len(output.metrics_msg.metrics_data), 1)

if __name__ == "__main__":
    unittest.main()
