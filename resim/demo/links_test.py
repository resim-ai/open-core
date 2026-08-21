# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import unittest

from httpx import URL

from resim.demo.links import (
    app_base_url,
    batch_url,
    compare_batches_url,
    dashboard_url,
    dashboards_url,
)

APP = "https://app.resim.ai"


class AppBaseUrlTest(unittest.TestCase):
    def test_production(self) -> None:
        self.assertEqual(app_base_url(URL("https://api.resim.ai/v1/")), APP)

    def test_staging(self) -> None:
        # Staging is api.resim.io / app.resim.io, so the same substitution has
        # to hold for it as for production.
        self.assertEqual(
            app_base_url(URL("https://api.resim.io/v1/")), "https://app.resim.io"
        )

    def test_only_the_leading_api_is_replaced(self) -> None:
        self.assertEqual(
            app_base_url(URL("https://api.api-team.resim.ai/v1/")),
            "https://app.api-team.resim.ai",
        )

    def test_keeps_the_scheme(self) -> None:
        self.assertEqual(
            app_base_url(URL("http://api.localhost/v1/")), "http://app.localhost"
        )

    def test_leaves_an_unrecognised_host_alone(self) -> None:
        self.assertEqual(
            app_base_url(URL("https://resim.example.com/v1/")),
            "https://resim.example.com",
        )


class RouteTest(unittest.TestCase):
    def test_batch(self) -> None:
        self.assertEqual(batch_url(APP, "p", "b"), f"{APP}/projects/p/batches/b")

    def test_compare(self) -> None:
        self.assertEqual(
            compare_batches_url(APP, "p", "a", "b"),
            f"{APP}/projects/p/batches/a/compare-batch/batch/b",
        )

    def test_dashboard(self) -> None:
        self.assertEqual(dashboard_url(APP, "p", "d"), f"{APP}/projects/p/dashboards/d")

    def test_dashboards_list(self) -> None:
        self.assertEqual(dashboards_url(APP, "p"), f"{APP}/projects/p/dashboards")


if __name__ == "__main__":
    unittest.main()
