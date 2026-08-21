# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import hashlib
import io
import json
import os
import tarfile
import tempfile
import unittest
from pathlib import Path
from typing import Optional
from unittest.mock import patch

from resim.demo import bundle
from resim.demo.bundle import Bundle, DemoDataError

MANIFEST = {
    "version": 1,
    "batches": {
        "a": {
            "name": "Baseline",
            "version": "v1.0.0",
            "jobs": [{"experience_name": "x"}],
        },
        "b": {"name": "Candidate", "version": "v1.1.0", "jobs": []},
    },
}


def _tarball(
    members: dict[str, bytes], unsafe: Optional[tarfile.TarInfo] = None
) -> bytes:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
        for name, payload in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            tar.addfile(info, io.BytesIO(payload))
        if unsafe is not None:
            tar.addfile(unsafe, io.BytesIO(b""))
    return buffer.getvalue()


def _valid_tarball() -> bytes:
    return _tarball(
        {
            "manifest.json": json.dumps(MANIFEST).encode(),
            "a/x/emissions.resim.jsonl": b'{"$metadata": {"topic": "t"}}\n',
        }
    )


class CacheDirTest(unittest.TestCase):
    def test_honours_xdg_cache_home(self) -> None:
        with patch.dict(os.environ, {"XDG_CACHE_HOME": "/somewhere/cache"}):
            self.assertEqual(
                bundle.cache_dir(),
                Path("/somewhere/cache") / "resim" / "sdk-demo" / bundle.BUNDLE_VERSION,
            )

    def test_falls_back_to_dot_cache(self) -> None:
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(
                bundle.cache_dir(),
                Path.home() / ".cache" / "resim" / "sdk-demo" / bundle.BUNDLE_VERSION,
            )

    def test_is_versioned_so_a_stale_bundle_is_never_reused(self) -> None:
        self.assertIn(bundle.BUNDLE_VERSION, str(bundle.cache_dir()))


class LoadTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_loads_manifest(self) -> None:
        (self.root / "manifest.json").write_text(json.dumps(MANIFEST))
        loaded = bundle.load(self.root)
        self.assertEqual(loaded.manifest["version"], 1)
        self.assertEqual(loaded.root, self.root)

    def test_raises_when_manifest_missing(self) -> None:
        with self.assertRaises(DemoDataError) as ctx:
            bundle.load(self.root)
        self.assertIn("manifest.json", str(ctx.exception))

    def test_raises_on_malformed_manifest(self) -> None:
        (self.root / "manifest.json").write_text("not json")
        with self.assertRaises(DemoDataError):
            bundle.load(self.root)

    def test_raises_when_manifest_is_not_an_object(self) -> None:
        (self.root / "manifest.json").write_text("[1, 2, 3]")
        with self.assertRaises(DemoDataError) as ctx:
            bundle.load(self.root)
        self.assertIn("not a JSON object", str(ctx.exception))


class BundleAccessorsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.bundle = Bundle(root=Path("/nowhere"), manifest=MANIFEST)

    def test_jobs_for_side(self) -> None:
        self.assertEqual(len(self.bundle.jobs("a")), 1)
        self.assertEqual(self.bundle.jobs("b"), [])

    def test_batch_metadata_for_side(self) -> None:
        self.assertEqual(self.bundle.batch("a")["version"], "v1.0.0")

    def test_unknown_side_raises_with_available_sides(self) -> None:
        with self.assertRaises(DemoDataError) as ctx:
            self.bundle.jobs("c")
        self.assertIn("['a', 'b']", str(ctx.exception))


class ExtractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.archive = self.root / "data.tar.gz"
        self.destination = self.root / "extracted"

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_extracts_members(self) -> None:
        self.archive.write_bytes(_valid_tarball())
        bundle._extract(self.archive, self.destination)
        self.assertTrue((self.destination / "manifest.json").is_file())
        self.assertTrue(
            (self.destination / "a" / "x" / "emissions.resim.jsonl").is_file()
        )

    def test_rejects_parent_traversal(self) -> None:
        self.archive.write_bytes(_tarball({"../escaped.txt": b"x"}))
        with self.assertRaises(DemoDataError) as ctx:
            bundle._extract(self.archive, self.destination)
        self.assertIn("unsafe path", str(ctx.exception))
        self.assertFalse(self.destination.exists())

    def test_rejects_absolute_paths(self) -> None:
        self.archive.write_bytes(_tarball({"/etc/passwd": b"x"}))
        with self.assertRaises(DemoDataError) as ctx:
            bundle._extract(self.archive, self.destination)
        self.assertIn("unsafe path", str(ctx.exception))

    def test_rejects_symlinks(self) -> None:
        link = tarfile.TarInfo("link")
        link.type = tarfile.SYMTYPE
        link.linkname = "/etc/passwd"
        self.archive.write_bytes(_tarball({"manifest.json": b"{}"}, unsafe=link))
        with self.assertRaises(DemoDataError) as ctx:
            bundle._extract(self.archive, self.destination)
        self.assertIn("not a regular file", str(ctx.exception))

    def test_replaces_an_existing_extraction(self) -> None:
        self.destination.mkdir()
        (self.destination / "stale.txt").write_text("old")
        self.archive.write_bytes(_valid_tarball())
        bundle._extract(self.archive, self.destination)
        self.assertFalse((self.destination / "stale.txt").exists())
        self.assertTrue((self.destination / "manifest.json").is_file())


class VerifyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.archive = Path(self.temp.name) / "data.tar.gz"
        self.archive.write_bytes(b"payload")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_accepts_matching_checksum(self) -> None:
        bundle._verify(self.archive, hashlib.sha256(b"payload").hexdigest())

    def test_raises_on_mismatch(self) -> None:
        with self.assertRaises(DemoDataError) as ctx:
            bundle._verify(self.archive, "0" * 64)
        self.assertIn("checksum", str(ctx.exception))

    def test_skips_when_no_checksum_pinned(self) -> None:
        bundle._verify(self.archive, "")


class EnsureTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_explicit_data_dir_skips_download(self) -> None:
        (self.root / "manifest.json").write_text(json.dumps(MANIFEST))
        with patch.object(bundle, "_download") as download:
            loaded = bundle.ensure(self.root)
        download.assert_not_called()
        self.assertEqual(loaded.root, self.root)

    def test_warm_cache_skips_download(self) -> None:
        cache = self.root / "cache"
        cache.mkdir()
        (cache / "manifest.json").write_text(json.dumps(MANIFEST))
        with (
            patch.object(bundle, "cache_dir", return_value=cache),
            patch.object(bundle, "_download") as download,
        ):
            bundle.ensure()
        download.assert_not_called()

    def test_cold_cache_downloads_verifies_and_extracts(self) -> None:
        cache = self.root / "cache"
        payload = _valid_tarball()

        def fake_download(url: str, destination: Path) -> None:
            destination.write_bytes(payload)

        with (
            patch.object(bundle, "cache_dir", return_value=cache),
            patch.object(bundle, "_download", side_effect=fake_download),
            patch.object(bundle, "BUNDLE_SHA256", hashlib.sha256(payload).hexdigest()),
        ):
            loaded = bundle.ensure()

        self.assertEqual(loaded.manifest["version"], 1)
        self.assertFalse(
            (cache.parent / f"{cache.name}.tar.gz").exists(),
            "the downloaded archive should be cleaned up after extraction",
        )

    def test_download_failure_surfaces_the_url(self) -> None:
        cache = self.root / "cache"
        with (
            patch.object(bundle, "cache_dir", return_value=cache),
            patch("httpx.stream", side_effect=__import__("httpx").ConnectError("nope")),
        ):
            with self.assertRaises(DemoDataError) as ctx:
                bundle.ensure()
        self.assertIn(bundle.BUNDLE_URL, str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
