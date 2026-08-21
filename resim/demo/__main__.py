# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Command line entry point for the ReSim SDK demo."""

import argparse
import sys

from resim.demo.bundle import DemoDataError
from resim.demo.run import DEFAULT_BRANCH, DEFAULT_PROJECT_NAME, run


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="resim-demo",
        description=(
            "Populate a ReSim project with two comparable batches of tests and "
            "a trends dashboard, then print links to the results."
        ),
    )
    parser.add_argument(
        "--project-name",
        default=DEFAULT_PROJECT_NAME,
        help=(
            "Project to run in, created if it does not exist. "
            f"Defaults to {DEFAULT_PROJECT_NAME!r}."
        ),
    )
    parser.add_argument(
        "--branch",
        default=DEFAULT_BRANCH,
        help=f"Branch to create the batches on. Defaults to {DEFAULT_BRANCH!r}.",
    )
    parser.add_argument(
        "--data-dir",
        default=None,
        help=(
            "Replay an already-extracted demo data bundle from this directory "
            "instead of downloading one."
        ),
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Only report failures.",
    )
    args = parser.parse_args()

    try:
        run(
            args.project_name,
            branch=args.branch,
            data_dir=args.data_dir,
            quiet=args.quiet,
        )
    except DemoDataError as e:
        print(f"resim-demo: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
