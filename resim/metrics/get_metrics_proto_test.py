# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit test for get_metrics_proto.
"""

import typing
import unittest
from dataclasses import dataclass
from http import HTTPStatus
from unittest.mock import patch
from google.protobuf.message import Message

from resim.metrics import get_metrics_proto


@dataclass
class MockMessage(Message):
    """
    A simple mock of a protobuf message.
    """
    data: str = ""

    # pylint: disable-next=invalid-name
    def ParseFromString(self, data: typing.Any) -> typing.Any:
        """This message type's serialization matches its data field."""
        self.data = data


@dataclass
class MockResponse:
    """
    A mock of a response from requests.get()
    """
    status_code: HTTPStatus
    content: str


_RESPONSE_MAP = {
    f"https://www.example.com/metric_{i}.binproto": f"data_{i}" for i in range(10)
}


def mocked_get(url: str) -> MockResponse:
    """
    The mock for our get request.
    """
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
    """
    The unit test case.
    """
    @patch("requests.Session")
    def test_get_metrics_proto(self, session: unittest.mock.MagicMock) -> None:
        """
        Test that we can fetch and deserialize protobuf with a
        mocked message and requests.Session.get()
        """
        session.get = mocked_get
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
