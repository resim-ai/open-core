# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Test for fetch_all_pages.
"""

import unittest
from dataclasses import dataclass

from resim.metrics.fetch_all_pages import async_fetch_all_pages, fetch_all_pages

_TEST_ARG = "some_arg"


@dataclass
class TestResponseType:
    """A simple response type for testing purposes"""

    value: str
    next_page_token: str


_START_TOKEN = "start"
_PAGE_TOKENS = {
    _START_TOKEN: "a",
    "a": "b",
    "b": "c",
    "c": "",
}


def _test_endpoint(
    test_arg: str, *, testcase: unittest.TestCase, page_token: str = ""
) -> TestResponseType:
    """A simple endpoint mock for testing purposes"""
    if not page_token:
        page_token = _START_TOKEN
    new_token = _PAGE_TOKENS[page_token]
    testcase.assertEqual(test_arg, _TEST_ARG)
    return TestResponseType(value=test_arg, next_page_token=new_token)


async def _async_test_endpoint(
    test_arg: str, *, testcase: unittest.TestCase, page_token: str = ""
) -> TestResponseType:
    """A simple endpoint mock for testing purposes"""
    if not page_token:
        page_token = _START_TOKEN
    new_token = _PAGE_TOKENS[page_token]
    testcase.assertEqual(test_arg, _TEST_ARG)
    return TestResponseType(value=test_arg, next_page_token=new_token)


class FetchAllPagesTest(unittest.TestCase):
    """The unittest case itself"""

    def test_fetch_all_pages(self) -> None:
        """Test that we fetch all pages we expect to"""
        all_pages: list[TestResponseType] = fetch_all_pages(
            _test_endpoint, _TEST_ARG, testcase=self
        )
        self.assertEqual(len(all_pages), len(_PAGE_TOKENS))
        for page in all_pages:
            self.assertEqual(page.value, _TEST_ARG)


class AsyncFetchAllPagesTest(unittest.IsolatedAsyncioTestCase):
    async def test_async_fetch_all_pages(self) -> None:
        """Test that we fetch all pages we expect to"""
        all_pages: list[TestResponseType] = await async_fetch_all_pages(
            _async_test_endpoint, _TEST_ARG, testcase=self
        )
        self.assertEqual(len(all_pages), len(_PAGE_TOKENS))
        for page in all_pages:
            self.assertEqual(page.value, _TEST_ARG)


if __name__ == "__main__":
    unittest.main()
