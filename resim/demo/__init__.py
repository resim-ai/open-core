# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A runnable tour of the ReSim platform.

One call populates a project with two comparable batches of tests and a trends
dashboard, then prints links to the results::

    from resim.demo import run

    run()

Or from a shell, after ``pip install resim-open-core``::

    resim-demo
"""

from resim.demo.bundle import DemoDataError
from resim.demo.run import DemoResult, run

__all__ = ["DemoDataError", "DemoResult", "run"]
