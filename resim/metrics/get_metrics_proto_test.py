# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit test for get_metrics_proto.
"""


import unittest
from dataclasses import dataclass
from http import HTTPStatus
from unittest.mock import patch

import resim.metrics.get_metrics_proto as get_metrics_proto

@dataclass
class MockMessage:
    data: str = ""

    def ParseFromString(self, data: str):
        self.data = data

@dataclass
class MockResponse:
    status_code: HTTPStatus
    content: str

_RESPONSE_MAP = {
    f"https://www.example.com/metric_{i}.binproto" : f"data_{i}" for i in range(10)
}

class MockSession:
    def get(self, url: str):
        if url in _RESPONSE_MAP:
            return MockResponse(
                status_code=HTTPStatus.OK,
                content=_RESPONSE_MAP[url],
            )
        return MockResponse(
            status_code=HTTPStatus.NOT_FOUND,
            content="",
        )



@patch("resim.metrics.get_metrics_proto.Message", new=MockMessage)
@patch("requests.Session", new=MockSession)
class GetMetricsProtoTest(unittest.TestCase):
    def test_get_metrics_proto(self):
        session = MockSession()
        message_type = MockMessage
        for url, data in _RESPONSE_MAP.items():
            self.assertEqual(get_metrics_proto.get_metrics_proto(message_type=message_type,
                                  session=session,
                                                                 url=url).data, data)
                                  

if __name__ == "__main__":
    unittest.main()
