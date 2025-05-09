# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import copy
import unittest
import uuid
from itertools import product
from random import Random

import numpy as np

import resim.metrics.proto.metrics_pb2 as mp
import resim.metrics.python.metrics_utils as mu


class MetricsUtilsTest(unittest.TestCase):
    def setUp(self) -> None:
        self._random = Random(933)

    def test_output_ctor(self) -> None:
        output = mu.ResimMetricsOutput()
        self.assertEqual(output.metrics_msg, mp.JobMetrics())
        self.assertEqual(output.packed_ids, set())

    def test_timestamp_to_int(self) -> None:
        ts = mu.Timestamp(secs=12, nanos=345)
        self.assertEqual(ts.to_nanos(), 12 * 1_000_000_000 + 345)

    def test_pack_timestamp(self) -> None:
        # SETUP
        test_ts = mu.Timestamp(
            secs=self._random.randrange(0, 1000000000),
            nanos=self._random.randrange(0, 1000000000),
        )

        # ACTION
        packed = test_ts.pack()

        # VERIFICATION
        self.assertEqual(test_ts.secs, packed.seconds)
        self.assertEqual(test_ts.nanos, packed.nanos)
        self.assertEqual(test_ts, mu.Timestamp.unpack(packed))

    def test_histogram_bucket_pack(self) -> None:
        # SETUP
        test_bucket = mu.HistogramBucket(
            lower=self._random.uniform(-10.0, -5.0),
            upper=self._random.uniform(5.0, 10.0),
        )

        # ACTION
        packed = test_bucket.pack()

        # VERIFICATION
        self.assertEqual(test_bucket.lower, packed.lower)
        self.assertEqual(test_bucket.upper, packed.upper)

    def test_double_failure_definition_pack(self) -> None:
        # SETUP
        test_failure_definition = mu.DoubleFailureDefinition(
            fails_above=self._random.uniform(-10.0, -5.0),
            fails_below=self._random.uniform(5.0, 10.0),
        )

        # ACTION
        packed = test_failure_definition.pack()

        # VERIFICATION
        self.assertEqual(test_failure_definition.fails_above, packed.fails_above)
        self.assertEqual(test_failure_definition.fails_below, packed.fails_below)

        # None branches
        for attr in ("fails_above", "fails_below"):
            modified_failure_definition = copy.copy(test_failure_definition)
            setattr(modified_failure_definition, attr, None)
            modified_packed = modified_failure_definition.pack()
            self.assertFalse(modified_packed.HasField(attr))

    def test_pack_uuid_to_proto(self) -> None:
        # SETUP
        test_uuid = uuid.uuid4()

        # ACTION
        packed = mu.pack_uuid_to_proto(test_uuid)

        # VERIFICATION
        self.assertEqual(uuid.UUID(packed.data), test_uuid)

    def test_pack_uuid_to_metric_id(self) -> None:
        # SETUP
        test_uuid = uuid.uuid4()

        # ACTION
        packed = mu.pack_uuid_to_metric_id(test_uuid)

        # VERIFICATION
        self.assertEqual(uuid.UUID(packed.id.data), test_uuid)

    def test_pack_series_to_proto(self) -> None:
        # SETUP
        inputs = {
            "empty": np.array([]),
            "float": np.array([1.0, 2.0, 3.0]),
            "timestamp": np.array(
                [mu.Timestamp(secs=i, nanos=3 * i) for i in range(3)]
            ),
            "uuid": np.array([uuid.uuid4() for _ in range(3)]),
            "string": np.array([str(i) for i in range(3)]),
            "status": np.array(
                [
                    mu.MetricStatus.PASSED_METRIC_STATUS,
                    mu.MetricStatus.FAIL_WARN_METRIC_STATUS,
                    mu.MetricStatus.PASSED_METRIC_STATUS,
                ]
            ),
            "class": np.array([mp.JobMetrics()]),
        }

        # Empty
        array_type, packed = mu.pack_series_to_proto(inputs["empty"], indexed=False)
        self.assertEqual(array_type, mp.MetricsDataType.Value("NO_DATA_TYPE"))
        self.assertEqual(packed, mp.Series())

        expected_types = {
            "float": "DOUBLE_SERIES_DATA_TYPE",
            "timestamp": "TIMESTAMP_SERIES_DATA_TYPE",
            "uuid": "UUID_SERIES_DATA_TYPE",
            "string": "STRING_SERIES_DATA_TYPE",
            "status": "METRIC_STATUS_SERIES_DATA_TYPE",
        }

        for (name, series_type), is_indexed in product(
            expected_types.items(), [False, True]
        ):
            array_type, packed = mu.pack_series_to_proto(
                inputs[name], indexed=is_indexed
            )
            prefix = "INDEXED_" if is_indexed else ""
            self.assertEqual(array_type, mp.MetricsDataType.Value(prefix + series_type))
            for packed_val, unpacked_val in zip(packed.doubles.series, inputs[name]):
                self.assertEqual(packed_val, unpacked_val)

        with self.assertRaises(ValueError):
            mu.pack_series_to_proto(inputs["class"], indexed=False)

    def test_timestamp_ordering(self) -> None:
        t1 = mu.Timestamp(
            secs=self._random.randrange(0, 1000000000),
            nanos=self._random.randrange(0, 1000000000),
        )
        t2 = mu.Timestamp(
            secs=t1.secs + 1,
            nanos=self._random.randrange(0, 1000000000),
        )
        self.assertLess(t1, t2)

    def test_pack_tags(self) -> None:
        # SETUP
        test_tag = mu.Tag("key", "value")

        # ACTION
        packed = test_tag.pack()

        # VERIFICATION
        self.assertEqual(packed.key, test_tag.key)
        self.assertEqual(packed.value, test_tag.value)
        self.assertEqual(mu.Tag.unpack(packed), test_tag)


if __name__ == "__main__":
    unittest.main()
