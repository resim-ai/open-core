# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Checks on the shipped metrics config.

The ReSim BFF validates a metrics config when it is synced, which means a
mistake in the demo's config would surface on someone's first run rather than
in CI. These tests mirror the rules that matter so it surfaces here instead.
"""

import re
import unittest
from importlib import resources
from typing import Any

import yaml

# System templates the platform renders. Kept in sync with the metrics guide's
# "Metric Templates" section.
SYSTEM_TEMPLATES = frozenset(
    {
        "bar",
        "histogram",
        "image",
        "line",
        "pie",
        "scalar",
        "state_timeline",
        "table",
        "video",
    }
)

METRIC_TYPES = frozenset({"test", "batch", "dashboard"})

# Tables the platform provides itself. A topic may not shadow these, and a
# query may reference them without declaring them.
BUILTIN_TABLES = frozenset({"metadata", "container_performance", "test_length_seconds"})


def _load() -> dict[str, Any]:
    config = resources.files("resim.demo") / "data" / "config.resim.yml"
    return yaml.safe_load(config.read_text(encoding="utf8"))


def _templates() -> set[str]:
    directory = resources.files("resim.demo") / "data" / "templates"
    return {
        entry.name for entry in directory.iterdir() if entry.name.endswith(".liquid")
    }


def _ctes_in(query: str) -> set[str]:
    """Names a query defines for itself with WITH ... AS (...)."""
    return {
        match.group(1)
        for match in re.finditer(
            r"(?:\bWITH\s+|,\s*)([a-zA-Z_][a-zA-Z0-9_]*)\s+AS\s*\(",
            query,
            re.IGNORECASE,
        )
    }


def _tables_in(query: str) -> set[str]:
    """Tables a query reads that it did not define itself.

    FROM and JOIN clauses, minus the query's own common table expressions.
    """
    read = {
        match.group(1)
        for match in re.finditer(
            r"\b(?:FROM|JOIN)\s+([a-zA-Z_][a-zA-Z0-9_]*)", query, re.IGNORECASE
        )
    }
    return read - _ctes_in(query)


class ConfigTest(unittest.TestCase):
    config: dict[str, Any]
    topics: dict[str, Any]
    metrics: dict[str, Any]
    metrics_sets: dict[str, Any]
    dashboards: dict[str, Any]

    @classmethod
    def setUpClass(cls) -> None:
        cls.config = _load()
        cls.topics = cls.config.get("topics") or {}
        cls.metrics = cls.config.get("metrics") or {}
        cls.metrics_sets = cls.config.get("metrics sets") or {}
        cls.dashboards = cls.config.get("dashboards") or {}

    def test_has_a_version(self) -> None:
        self.assertEqual(self.config.get("version"), 1)

    def test_metrics_sets_reference_known_metrics(self) -> None:
        for set_name, body in self.metrics_sets.items():
            for metric in body.get("metrics") or []:
                self.assertIn(
                    metric,
                    self.metrics,
                    f"metrics set {set_name!r} references unknown metric {metric!r}",
                )

    def test_every_metric_belongs_to_a_set(self) -> None:
        used = {
            metric
            for body in self.metrics_sets.values()
            for metric in body.get("metrics") or []
        }
        self.assertEqual(
            set(self.metrics) - used,
            set(),
            "these metrics are defined but never run",
        )

    def test_dashboards_reference_known_metrics_sets(self) -> None:
        for name, body in self.dashboards.items():
            self.assertIn(
                body.get("metrics_set"),
                self.metrics_sets,
                f"dashboard {name!r} references an unknown metrics set",
            )

    def test_dashboard_metrics_sets_hold_only_dashboard_metrics(self) -> None:
        # A dashboard's queries run across batches; test and batch metrics do
        # not, so mixing them into a dashboard's set renders nothing useful.
        for name, body in self.dashboards.items():
            set_name = body["metrics_set"]
            for metric in self.metrics_sets[set_name].get("metrics") or []:
                self.assertEqual(
                    self.metrics[metric].get("type"),
                    "dashboard",
                    f"{metric!r} is in dashboard {name!r}'s metrics set but is "
                    "not a dashboard metric",
                )

    def test_batch_metrics_sets_hold_no_dashboard_metrics(self) -> None:
        dashboard_sets = {body["metrics_set"] for body in self.dashboards.values()}
        for set_name, body in self.metrics_sets.items():
            if set_name in dashboard_sets:
                continue
            for metric in body.get("metrics") or []:
                self.assertNotEqual(
                    self.metrics[metric].get("type"),
                    "dashboard",
                    f"dashboard metric {metric!r} is in {set_name!r}, which is "
                    "run against a batch",
                )

    def test_metric_types_are_known(self) -> None:
        for name, metric in self.metrics.items():
            self.assertIn(metric.get("type"), METRIC_TYPES, f"metric {name!r}")

    def test_system_templates_are_known(self) -> None:
        for name, metric in self.metrics.items():
            if metric.get("template_type") == "system":
                self.assertIn(
                    metric.get("template"), SYSTEM_TEMPLATES, f"metric {name!r}"
                )

    def test_custom_templates_are_shipped(self) -> None:
        available = _templates()
        for name, metric in self.metrics.items():
            if metric.get("template_type") == "custom":
                self.assertIn(
                    metric.get("template_file"),
                    available,
                    f"metric {name!r} references a template that is not shipped",
                )

    def test_every_shipped_template_is_used(self) -> None:
        referenced = {
            metric.get("template_file")
            for metric in self.metrics.values()
            if metric.get("template_type") == "custom"
        }
        self.assertEqual(_templates() - referenced, set())

    def test_metrics_declare_a_template_and_a_query(self) -> None:
        for name, metric in self.metrics.items():
            self.assertIn(
                metric.get("template_type"), {"system", "custom"}, f"metric {name!r}"
            )
            self.assertTrue(metric.get("query_string"), f"metric {name!r} has no query")

    def test_topics_do_not_shadow_builtin_tables(self) -> None:
        self.assertEqual(set(self.topics) & BUILTIN_TABLES, set())

    def test_at_most_one_image_and_one_video_column_per_topic(self) -> None:
        for name, topic in self.topics.items():
            types = list((topic.get("schema") or {}).values())
            self.assertLessEqual(types.count("image"), 1, f"topic {name!r}")
            self.assertLessEqual(types.count("video"), 1, f"topic {name!r}")

    def test_no_topic_mixes_image_and_video(self) -> None:
        for name, topic in self.topics.items():
            types = set((topic.get("schema") or {}).values())
            self.assertFalse(
                {"image", "video"} <= types,
                f"topic {name!r} declares both an image and a video column",
            )

    def test_media_metrics_have_a_matching_topic_column(self) -> None:
        for column in ("image", "video"):
            uses_column = any(
                column in (topic.get("schema") or {}).values()
                for topic in self.topics.values()
            )
            metrics = [
                name
                for name, metric in self.metrics.items()
                if metric.get("template_type") == "system"
                and metric.get("template") == column
            ]
            if metrics:
                self.assertTrue(
                    uses_column,
                    f"{metrics} use the {column} template but no topic declares "
                    f"a {column} column",
                )

    def test_queries_only_read_declared_topics(self) -> None:
        known = set(self.topics) | BUILTIN_TABLES
        for name, metric in self.metrics.items():
            queries = [metric["query_string"]]
            status = metric.get("status")
            if status:
                queries.append(status["query_string"])
            for query in queries:
                for table in _tables_in(query):
                    self.assertIn(
                        table,
                        known,
                        f"metric {name!r} reads from undeclared table {table!r}",
                    )

    def test_status_checks_take_one_threshold_parameter(self) -> None:
        for name, metric in self.metrics.items():
            status = metric.get("status")
            if not status:
                continue
            self.assertEqual(
                status["query_string"].count("?"),
                1,
                f"metric {name!r} status query must have exactly one ? parameter",
            )
            self.assertIn(
                "block", status, f"metric {name!r} status needs a block value"
            )

    def test_covers_every_system_template(self) -> None:
        # The demo exists to show what the platform can render, so a template
        # going unused is a gap in the demo rather than a config error.
        used = {
            metric.get("template")
            for metric in self.metrics.values()
            if metric.get("template_type") == "system"
        }
        self.assertEqual(SYSTEM_TEMPLATES - used, set())

    def test_covers_test_batch_and_dashboard_metrics(self) -> None:
        self.assertEqual(
            METRIC_TYPES - {metric["type"] for metric in self.metrics.values()}, set()
        )

    def test_declares_an_event_topic(self) -> None:
        self.assertTrue(
            any(topic.get("event") for topic in self.topics.values()),
            "the demo should populate the Events tab",
        )


if __name__ == "__main__":
    unittest.main()
