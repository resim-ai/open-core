# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A runnable tour of the ReSim platform.

One call populates a project with two comparable batches of tests and a trends
dashboard, then prints links to the results::

    from signalflag.demo import run

    run()

Or from a shell, after ``pip install signalflag-sdk``::

    signalflag-demo

``resim-demo`` is an alias for the same command.
"""

from signalflag.demo.bundle import DemoDataError
from signalflag.demo.run import (
    DEMOS,
    Demo,
    DemoResult,
    config_path,
    run,
    templates_path,
)

__all__ = [
    "DEMOS",
    "Demo",
    "DemoDataError",
    "DemoResult",
    "config_path",
    "run",
    "templates_path",
]
