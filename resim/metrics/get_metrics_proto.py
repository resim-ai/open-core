# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
This module contains a single function for getting a protobuf message from a
URL.
"""

import typing
from http import HTTPStatus

import requests
from google.protobuf.message import Message

T = typing.TypeVar("T", bound=Message)


def get_metrics_proto(
    *, message_type: type[T], session: requests.Session, url: str
) -> T:
    """
    Get a metrics protobuf message from a url.

    This function gets a protobuf message of type message_type from
    the given url using the given requests Session.
    """
    response = session.get(url)
    assert response.status_code == HTTPStatus.OK
    message: T = message_type()
    message.ParseFromString(response.content)
    return message
