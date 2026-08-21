# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Tests for the command line entry point.

`resim-demo` is what most people will actually invoke, so its argument wiring
and exit codes are worth pinning: a demo that exits 0 on a failure, or ignores
--project-name, misleads whoever ran it.
"""

import unittest
from importlib import import_module
from unittest.mock import MagicMock, patch

from resim.demo.bundle import DemoDataError
from resim.demo.run import DEFAULT_BRANCH, DEFAULT_PROJECT_NAME

main_module = import_module("resim.demo.__main__")


class MainTest(unittest.TestCase):
    def _main(self, *argv: str) -> tuple[int, MagicMock]:
        with (
            patch.object(main_module, "run") as run,
            patch("sys.argv", ["resim-demo", *argv]),
        ):
            code = main_module.main()
        return code, run

    def test_returns_zero_on_success(self) -> None:
        code, _ = self._main()
        self.assertEqual(code, 0)

    def test_defaults_match_the_library_defaults(self) -> None:
        # The CLI and `run()` disagreeing about where the demo lands would be a
        # nasty surprise, so the defaults are asserted rather than duplicated.
        _, run = self._main()
        run.assert_called_once()
        self.assertEqual(run.call_args.args[0], DEFAULT_PROJECT_NAME)
        self.assertEqual(run.call_args.kwargs["branch"], DEFAULT_BRANCH)
        self.assertIsNone(run.call_args.kwargs["data_dir"])
        self.assertFalse(run.call_args.kwargs["quiet"])

    def test_passes_every_option_through(self) -> None:
        _, run = self._main(
            "--project-name",
            "other",
            "--branch",
            "b",
            "--data-dir",
            "/tmp/d",
            "--quiet",
        )
        self.assertEqual(run.call_args.args[0], "other")
        self.assertEqual(run.call_args.kwargs["branch"], "b")
        self.assertEqual(run.call_args.kwargs["data_dir"], "/tmp/d")
        self.assertTrue(run.call_args.kwargs["quiet"])

    def test_reports_missing_data_and_exits_non_zero(self) -> None:
        with (
            patch.object(
                main_module,
                "run",
                side_effect=DemoDataError("no data at https://example/x"),
            ),
            patch("sys.argv", ["resim-demo"]),
            patch("sys.stderr") as stderr,
        ):
            code = main_module.main()

        self.assertEqual(code, 1)
        written = "".join(str(c.args[0]) for c in stderr.write.call_args_list if c.args)
        self.assertIn("https://example/x", written)

    def test_other_exceptions_are_not_swallowed(self) -> None:
        # Only the data error is turned into a message; anything else should
        # keep its traceback rather than being reported as a tidy failure.
        with (
            patch.object(main_module, "run", side_effect=RuntimeError("boom")),
            patch("sys.argv", ["resim-demo"]),
        ):
            with self.assertRaises(RuntimeError):
                main_module.main()

    def test_help_exits_cleanly(self) -> None:
        with patch("sys.argv", ["resim-demo", "--help"]), patch("sys.stdout"):
            with self.assertRaises(SystemExit) as ctx:
                main_module.main()
        self.assertEqual(ctx.exception.code, 0)


if __name__ == "__main__":
    unittest.main()
