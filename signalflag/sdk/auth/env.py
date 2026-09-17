# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Environment variables the SDK reads.

Every variable has a ``SIGNALFLAG_`` name and a legacy ``RESIM_`` name from
before the SDK was rebranded. :func:`getenv` prefers the new name and falls
back to the old one, so existing shells, CI secrets and ``.env`` files keep
working while new documentation only has to mention ``SIGNALFLAG_*``.
"""

import os
from typing import Optional

__all__ = [
    "API_URL",
    "AUTH_DOMAIN",
    "CLIENT_ID",
    "PASSWORD",
    "USERNAME",
    "describe",
    "getenv",
    "legacy_name",
]

PREFIX = "SIGNALFLAG_"
LEGACY_PREFIX = "RESIM_"

USERNAME = "SIGNALFLAG_USERNAME"
PASSWORD = "SIGNALFLAG_PASSWORD"
API_URL = "SIGNALFLAG_API_URL"
AUTH_DOMAIN = "SIGNALFLAG_AUTH_DOMAIN"
CLIENT_ID = "SIGNALFLAG_CLIENT_ID"


def legacy_name(name: str) -> str:
    """The pre-rebrand spelling of a ``SIGNALFLAG_*`` variable: ``RESIM_*``."""
    if not name.startswith(PREFIX):
        raise ValueError(f"expected a {PREFIX}* variable name, got {name!r}")
    return LEGACY_PREFIX + name[len(PREFIX) :]


def getenv(name: str, default: Optional[str] = None) -> Optional[str]:
    """Read ``name``, falling back to its legacy ``RESIM_`` spelling.

    A variable that is set but empty counts as unset, so an empty
    ``SIGNALFLAG_USERNAME`` does not hide a populated ``RESIM_USERNAME``.
    """
    return os.environ.get(name) or os.environ.get(legacy_name(name)) or default


def describe(name: str) -> str:
    """Both spellings of a variable, for error messages and help text."""
    return f"{name} (or {legacy_name(name)})"
