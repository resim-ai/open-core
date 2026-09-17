# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from datetime import datetime, timedelta, timezone

_KEY = "expires_at"


def add_expiration_time(*, token_data: dict) -> None:
    """Adds the "expires_at" key to token data.

    Adds an "expires_at" key computed from the current time and the
    "expires_in" key in the token_data dict.

    Args:
      token_data: A token data dict, as returned upon successful
                  resolution of the device code flow.
    """
    DURATION_KEY = "expires_in"
    if DURATION_KEY not in token_data:
        raise ValueError("No expiration lifetime in token data")
    token_data[_KEY] = (
        datetime.now(timezone.utc) + timedelta(seconds=token_data[DURATION_KEY])
    ).isoformat()


def is_expired(*, token_data: dict) -> bool:
    """Checks whether the given token is expired.

    Defaults to true if the "expires_at" key is not present in the
    token_data. Otherwise, returns expired if we're within 1 hour of
    the nominal expiration time.

    Args:
      token_data: A token data dict, as returned upon successful
                  resolution of the device code flow.
    """
    if _KEY not in token_data:
        return True
    return datetime.now(timezone.utc) + timedelta(hours=1) > datetime.fromisoformat(
        token_data[_KEY]
    )
