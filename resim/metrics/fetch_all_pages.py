# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
This module contains a simple function for fetching all pages from a given
endpoint.

In ReSim's API we have a number of endpoints with paged responses. These work
like so: when first queried, they return a page of the specified (or default)
length and a token for the next page. This token can be passed to the same
endpoint to get the next page of the same size. This can be repeated until the
returned page token is nil. From code, we often just want to get the content
from all pages rather than inspecting things page by page. Therefore, this
function simply calls the given endpoint in a loop until no further pages are
returned.
"""

import typing


# pylint: disable-next=too-few-public-methods
class HasNextPageToken(typing.Protocol):
    """A simple protocol for classes having the next_page_token field"""

    next_page_token: str


# pylint: disable-next=invalid-name
ResponseType = typing.TypeVar("ResponseType", bound=HasNextPageToken)


def fetch_all_pages(
    endpoint: typing.Callable[..., ResponseType],
    *args: typing.Any,
    **kwargs: typing.Any
) -> list[ResponseType]:
    """
    Fetches all pages from a given endpoint.
    """
    responses = []
    responses.append(endpoint(*args, **kwargs))
    assert responses[-1] is not None

    page_token = responses[-1].next_page_token
    while page_token:
        responses.append(endpoint(*args, **kwargs, page_token=page_token))
        assert responses[-1] is not None
        page_token = responses[-1].next_page_token
    return responses


async def async_fetch_all_pages(
    endpoint: typing.Callable[..., typing.Awaitable[ResponseType]],
    *args: typing.Any,
    **kwargs: typing.Any
) -> list[ResponseType]:
    """
    Fetches all pages from a given endpoint.
    """
    responses = []
    responses.append(await endpoint(*args, **kwargs))
    assert responses[-1] is not None

    page_token = responses[-1].next_page_token
    while page_token:
        responses.append(await endpoint(*args, **kwargs, page_token=page_token))
        assert responses[-1] is not None
        page_token = responses[-1].next_page_token
    return responses
