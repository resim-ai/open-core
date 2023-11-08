

import google.protobuf
import requests
import uuid
import typing
from http import HTTPStatus

T = typing.TypeVar("T", bound=google.protobuf.message.Message)

def get_metrics_proto(*,
                   message_type: type[T],
                   session: requests.Session,
                   url: str) -> T:
    response = session.get(url)
    assert response.status_code == HTTPStatus.OK
    message: T = message_type()
    message.ParseFromString(response.content)
    return message
