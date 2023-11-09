# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit test for get_metrics_proto.
"""

import typing
import requests
import unittest
from dataclasses import dataclass
from http import HTTPStatus
from unittest.mock import patch
from google.protobuf.message import Message

import resim.metrics.get_metrics_proto as get_metrics_proto


@dataclass
class MockMessage(Message):
    data: str = ""

    def ParseFromString(self, data: typing.Any) -> typing.Any:
        self.data = data


@dataclass
class MockResponse:
    status_code: HTTPStatus
    content: str


_RESPONSE_MAP = {
    f"https://www.example.com/metric_{i}.binproto": f"data_{i}" for i in range(10)
}


def get(url: str) -> MockResponse:
    if url in _RESPONSE_MAP:
        return MockResponse(
            status_code=HTTPStatus.OK,
            content=_RESPONSE_MAP[url],
        )
    return MockResponse(
        status_code=HTTPStatus.NOT_FOUND,
        content="",
    )


class GetMetricsProtoTest(unittest.TestCase):
    @patch("requests.Session")
    def test_get_metrics_proto(self, session: unittest.mock.MagicMock) -> None:
        session.get = get
        message_type = MockMessage
        for url, data in _RESPONSE_MAP.items():
            self.assertEqual(
                get_metrics_proto.get_metrics_proto(
                    message_type=message_type,
                    session=session,
                    url=url).data,
                data)


if __name__ == "__main__":
    unittest.main()
