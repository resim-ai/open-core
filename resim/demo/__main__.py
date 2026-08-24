# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Command line entry point for the ReSim SDK demo."""

import argparse
import sys
import textwrap

import httpx

from resim.demo.bundle import DemoDataError
from resim.demo.run import DEMOS, run


def _list_demos() -> None:
    """Print the demos and how to pick one."""
    print("resim-demo replays real test data into your own ReSim project.\n")
    print("Pick one with --demo:\n")
    width = max(len(key) for key in DEMOS)
    for key in sorted(DEMOS):
        demo = DEMOS[key]
        first, *rest = textwrap.wrap(demo.summary, 62)
        print(f"  {key.ljust(width)}   {first}")
        for line in rest:
            print(f"  {' ' * width}   {line}")
    # Suggest the fuller tour rather than whichever key sorts first.
    suggested = "navigation" if "navigation" in DEMOS else sorted(DEMOS)[0]
    print(f"\nFor example:\n  resim-demo --demo {suggested}")


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="resim-demo",
        description=(
            "Populate a ReSim project with two comparable batches of tests and "
            "a trends dashboard, then print links to the results."
        ),
    )
    parser.add_argument(
        "--demo",
        default=None,
        choices=sorted(DEMOS),
        help="Which demo to run. Run with no arguments to see what each one is.",
    )
    parser.add_argument(
        "--project-name",
        default=None,
        help=(
            "Project to run in, created if it does not exist. Defaults to the "
            "chosen demo's own project name."
        ),
    )
    parser.add_argument(
        "--branch",
        default=None,
        help=(
            "Branch to create the batches on. Defaults to the chosen demo's own branch."
        ),
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

    if args.demo is None:
        # The demos are peers rather than one being the obvious choice, so
        # rather than silently picking, say what is on offer.
        _list_demos()
        return 0

    try:
        run(
            args.project_name,
            demo=args.demo,
            branch=args.branch,
            data_dir=args.data_dir,
            quiet=args.quiet,
        )
    except DemoDataError as e:
        print(f"resim-demo: {e}", file=sys.stderr)
        return 1
    except httpx.HTTPError as e:
        # Uploads retry on their own; reaching here means the network stayed
        # down. Report it rather than dumping a traceback on someone who is
        # only trying to look at the product.
        print(
            f"resim-demo: lost contact with ReSim ({e!r}).\n"
            "Any batches already created are in the app; re-run to start a "
            "fresh pair.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
