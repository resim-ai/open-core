# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Fetch the demo's replay data.

The demo replays real test data captured from two ReSim batches. That data is
too large to ship inside the wheel, so it lives as a single tarball in a public
bucket and is downloaded and cached on first run.
"""

import hashlib
import json
import os
import shutil
import sys
import tarfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Union

import httpx

__all__ = ["Bundle", "BundleSource", "DemoDataError", "ensure", "load"]


@dataclass(frozen=True)
class BundleSource:
    """Where one demo's replay data lives, and how to know it arrived intact.

    Spelled out per demo rather than derived from a naming convention, so what
    gets downloaded is greppable and a bundle can be republished under any name.
    """

    url: str
    sha256: str
    # Cache directory name. Carries the bundle's version so regenerated data
    # never collides with a stale copy of the old bundle.
    cache_key: str


MANIFEST_NAME = "manifest.json"

_DOWNLOAD_TIMEOUT_SECONDS = 300.0


class DemoDataError(RuntimeError):
    """The demo's replay data could not be fetched or is not usable."""


@dataclass(frozen=True)
class Bundle:
    """An extracted data bundle: where it lives, and what is in it."""

    root: Path
    manifest: dict[str, Any]

    def jobs(self, side: str) -> list[dict[str, Any]]:
        """The job entries for one side of the comparison (``"a"`` or ``"b"``)."""
        batches = self.manifest.get("batches") or {}
        batch = batches.get(side)
        if batch is None:
            raise DemoDataError(
                f"bundle manifest has no batch {side!r}; "
                f"found {sorted(batches)}. The bundle may be from an "
                "incompatible version."
            )
        return list(batch.get("jobs") or [])

    def batch(self, side: str) -> dict[str, Any]:
        """The manifest metadata for one side (name, version, description)."""
        return dict((self.manifest.get("batches") or {}).get(side) or {})


def cache_dir(source: Optional[BundleSource] = None) -> Path:
    """Directory a bundle is cached in, honouring ``XDG_CACHE_HOME``.

    Each demo caches under its own key, so switching demos does not re-download
    the one you were using before.
    """
    base = os.environ.get("XDG_CACHE_HOME")
    root = Path(base) if base else Path.home() / ".cache"
    cache = root / "resim" / "sdk-demo"
    return cache / source.cache_key if source is not None else cache


def load(directory: Union[str, Path]) -> Bundle:
    """Load an already-extracted bundle from a local directory.

    Raises:
        DemoDataError: If the directory has no readable manifest.
    """
    root = Path(directory)
    manifest_path = root / MANIFEST_NAME
    if not manifest_path.is_file():
        raise DemoDataError(
            f"no {MANIFEST_NAME} in {root}. Expected an extracted demo data bundle."
        )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf8"))
    except (OSError, json.JSONDecodeError) as e:
        raise DemoDataError(f"could not read {manifest_path}: {e}") from e
    if not isinstance(manifest, dict):
        raise DemoDataError(f"{manifest_path} is not a JSON object")
    return Bundle(root=root, manifest=manifest)


def ensure(
    source: BundleSource,
    data_dir: Optional[Union[str, Path]] = None,
) -> Bundle:
    """Return a demo's data bundle, downloading and caching it if needed.

    Args:
        source: Which bundle to fetch.
        data_dir: An already-extracted bundle to use instead of downloading.
            Useful for development and for air-gapped runs.

    Raises:
        DemoDataError: If the bundle cannot be downloaded, fails its checksum,
            or contains unsafe archive members.
    """
    if data_dir is not None:
        return load(data_dir)

    destination = cache_dir(source)
    if (destination / MANIFEST_NAME).is_file():
        return load(destination)

    archive = destination.parent / f"{destination.name}.tar.gz"
    archive.parent.mkdir(parents=True, exist_ok=True)
    _download(source.url, archive)
    try:
        _verify(archive, source.sha256, source.url)
        _extract(archive, destination)
    finally:
        archive.unlink(missing_ok=True)

    return load(destination)


def _download(url: str, destination: Path) -> None:
    try:
        with httpx.stream(
            "GET", url, follow_redirects=True, timeout=_DOWNLOAD_TIMEOUT_SECONDS
        ) as response:
            response.raise_for_status()
            with open(destination, "wb") as f:
                for chunk in response.iter_bytes():
                    f.write(chunk)
    except httpx.HTTPError as e:
        destination.unlink(missing_ok=True)
        raise DemoDataError(
            f"could not download the demo data from {url}: {e}. "
            "The demo needs this data to run; check your network access to "
            "that host and try again."
        ) from e


def _verify(archive: Path, expected: str, url: str) -> None:
    if not expected:
        return
    digest = hashlib.sha256()
    with open(archive, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    actual = digest.hexdigest()
    if actual != expected:
        raise DemoDataError(
            f"the demo data downloaded from {url} does not match its "
            f"expected checksum (got {actual}, expected {expected}). Refusing "
            "to use it."
        )


def _extract(archive: Path, destination: Path) -> None:
    """Extract the bundle, rejecting anything that could escape ``destination``.

    ``tarfile``'s own ``data`` filter does this, but it only exists from Python
    3.12 and the SDK supports 3.10, so the member checks are done here and the
    filter is passed as well wherever it is available.
    """
    staging = destination.with_name(destination.name + ".partial")
    if staging.exists():
        _rmtree(staging)
    staging.mkdir(parents=True)

    try:
        with tarfile.open(archive, "r:gz") as tar:
            for member in tar.getmembers():
                _check_member(member)
            # 3.12 deprecates extractall without a filter and 3.14 changes
            # its default, so ask for one where the parameter exists. The member
            # checks above are what covers 3.10 and 3.11, where it does not.
            if sys.version_info >= (3, 12):
                tar.extractall(staging, filter="data")  # pragma: no cover
            else:
                tar.extractall(staging)
    except (tarfile.TarError, OSError) as e:
        _rmtree(staging)
        raise DemoDataError(f"could not extract the demo data: {e}") from e

    if destination.exists():
        _rmtree(destination)
    staging.rename(destination)


def _check_member(member: tarfile.TarInfo) -> None:
    if not (member.isfile() or member.isdir()):
        raise DemoDataError(
            f"demo data contains {member.name!r}, which is not a regular file "
            "or directory. Refusing to extract it."
        )
    path = Path(member.name)
    if path.is_absolute() or ".." in path.parts:
        raise DemoDataError(
            f"demo data contains an unsafe path {member.name!r}. Refusing to "
            "extract it."
        )


def _rmtree(path: Path) -> None:
    shutil.rmtree(path, ignore_errors=True)
