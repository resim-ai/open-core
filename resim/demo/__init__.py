# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Deprecated alias for :mod:`signalflag.demo`.

The demo now lives at signalflag.demo. Importing resim.demo or any submodule
returns the corresponding signalflag.demo module object, so
from resim.demo import run keeps working. New code should import from
signalflag.demo directly.
"""

from resim._alias import alias_package

alias_package(__name__, "signalflag.demo")
