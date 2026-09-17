# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Deprecated alias for :mod:`signalflag.sdk`.

The SDK now lives at signalflag.sdk. Importing resim.sdk or any submodule
returns the corresponding signalflag.sdk module object, so existing code keeps
working unchanged. New code should import from signalflag.sdk directly.
"""

from resim._alias import alias_package

alias_package(__name__, "signalflag.sdk")
