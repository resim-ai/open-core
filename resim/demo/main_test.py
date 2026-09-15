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

import httpx

from resim.demo.bundle import DemoDataError
from resim.demo.run import DEMOS

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
        code, _ = self._main("--demo", "navigation")
        self.assertEqual(code, 0)

    def test_no_demo_lists_them_instead_of_picking_one(self) -> None:
        # The demos are peers. Running one the caller did not ask for would be
        # a surprise, and a bare error would not say what is available.
        with patch("sys.stdout") as stdout:
            code, run = self._main()
        run.assert_not_called()
        self.assertEqual(code, 0)
        printed = "".join(str(c.args[0]) for c in stdout.write.call_args_list if c.args)
        self.assertIn("--demo", printed)
        for key in DEMOS:
            self.assertIn(key, printed)

    def test_options_without_a_demo_are_an_error_not_a_listing(self) -> None:
        # Listing the demos and exiting 0 while dropping the argument reads as
        # success: a script would see the run succeed having done nothing.
        for argv in (
            ["--project-name", "my project"],
            ["--branch", "b"],
            ["--data-dir", "/tmp/d"],
            ["--quiet"],
        ):
            with self.subTest(argv=argv):
                with (
                    patch.object(main_module, "run") as run,
                    patch("sys.argv", ["resim-demo", *argv]),
                    patch("sys.stderr") as stderr,
                ):
                    with self.assertRaises(SystemExit) as ctx:
                        main_module.main()

                self.assertNotEqual(ctx.exception.code, 0)
                run.assert_not_called()
                written = "".join(
                    str(c.args[0]) for c in stderr.write.call_args_list if c.args
                )
                self.assertIn("--demo", written)
                self.assertIn(argv[0], written)

    def test_defaults_defer_to_the_chosen_demo(self) -> None:
        # The CLI passes None for project and branch so `run()` fills in the
        # demo's own values; hardcoding them here would silently drift when a
        # demo changes its project name.
        _, run = self._main("--demo", "navigation")
        run.assert_called_once()
        self.assertIsNone(run.call_args.args[0])
        self.assertIsNone(run.call_args.kwargs["branch"])
        self.assertEqual(run.call_args.kwargs["demo"], "navigation")
        self.assertIsNone(run.call_args.kwargs["data_dir"])
        self.assertFalse(run.call_args.kwargs["quiet"])

    def test_demo_flag_is_passed_through(self) -> None:
        _, run = self._main("--demo", "mujoco")
        self.assertEqual(run.call_args.kwargs["demo"], "mujoco")

    def test_unknown_demo_is_refused_by_argparse(self) -> None:
        with patch("sys.argv", ["resim-demo", "--demo", "nope"]), patch("sys.stderr"):
            with self.assertRaises(SystemExit) as ctx:
                main_module.main()
        self.assertNotEqual(ctx.exception.code, 0)

    def test_passes_every_option_through(self) -> None:
        _, run = self._main(
            "--demo",
            "navigation",
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
            patch("sys.argv", ["resim-demo", "--demo", "navigation"]),
            patch("sys.stderr") as stderr,
        ):
            code = main_module.main()

        self.assertEqual(code, 1)
        written = "".join(str(c.args[0]) for c in stderr.write.call_args_list if c.args)
        self.assertIn("https://example/x", written)

    def test_network_failure_is_reported_without_a_traceback(self) -> None:
        # Uploads retry internally, so reaching here means the network stayed
        # down. Someone trying to look at the product should get a sentence.
        with (
            patch.object(
                main_module, "run", side_effect=httpx.ConnectError("reset by peer")
            ),
            patch("sys.argv", ["resim-demo", "--demo", "navigation"]),
            patch("sys.stderr") as stderr,
        ):
            code = main_module.main()

        self.assertEqual(code, 1)
        written = "".join(str(c.args[0]) for c in stderr.write.call_args_list if c.args)
        self.assertIn("lost contact with ReSim", written)

    def test_unexpected_exceptions_are_not_swallowed(self) -> None:
        # A bug in the demo should keep its traceback rather than being
        # reported as a tidy failure.
        with (
            patch.object(main_module, "run", side_effect=RuntimeError("boom")),
            patch("sys.argv", ["resim-demo", "--demo", "navigation"]),
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
