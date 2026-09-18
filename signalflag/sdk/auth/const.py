# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Defaults for talking to the platform, and where the SDK keeps its state."""

import pathlib

DEFAULT_BASE_URL = "https://api.resim.ai/v1"
DEFAULT_DOMAIN = "https://resim.us.auth0.com"
DEFAULT_SCOPE = "offline_access"
DEFAULT_AUDIENCE = "https://api.resim.ai"

# The SDK keeps its token cache in a dot-directory under $HOME. It moved from
# .resim to .signalflag with the rebrand; an existing .resim is still honoured
# so nobody has to log in again, mirroring the signalflag CLI.
CONFIG_DIR_NAME = ".signalflag"
LEGACY_CONFIG_DIR_NAME = ".resim"
TOKEN_CACHE_FILE_NAME = "token.json"


def config_dir(home: pathlib.Path | None = None) -> pathlib.Path:
    """The directory holding the SDK's cached credentials.

    Prefers ``~/.signalflag``. Falls back to ``~/.resim`` only when that exists
    and ``~/.signalflag`` does not, so an upgrade keeps using the token it
    already has. When neither exists, ``~/.signalflag`` is returned and is
    created on first write.
    """
    home = pathlib.Path.home() if home is None else home
    current = home / CONFIG_DIR_NAME
    legacy = home / LEGACY_CONFIG_DIR_NAME
    if not current.exists() and legacy.is_dir():
        return legacy
    return current


def default_cache_location(home: pathlib.Path | None = None) -> pathlib.Path:
    """Where the auth clients cache their token unless told otherwise."""
    return config_dir(home) / TOKEN_CACHE_FILE_NAME


# Kept for callers that imported the old constant. It is resolved once at import
# time; prefer default_cache_location() for a value that reflects $HOME now.
DEFAULT_CACHE_LOCATION = default_cache_location()
