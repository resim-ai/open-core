# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


import argparse
import base64
import hashlib
import json
import re
from pathlib import Path

import httpx


def generate_sha256_hash(url: str) -> str:
    with httpx.Client(follow_redirects=True) as client:
        response = client.get(url)
        response.raise_for_status()
    sha256_digest = hashlib.sha256(response.content).digest()
    base64_encoded = base64.b64encode(sha256_digest).decode("utf-8")
    return f"sha256-{base64_encoded}"


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="update_open_core", description="Update the entries for open core"
    )

    parser.add_argument("--registry-dir", type=Path, required=True)
    parser.add_argument("--version", type=str, required=True)

    args = parser.parse_args()

    version = args.version.removeprefix("v")

    # Verify that the args all make sense and the right directories exist /
    # don't exist
    module_path = args.registry_dir / "modules" / "resim_open_core"
    assert module_path.is_dir(), "Bad module path!"

    new_entry_dir = module_path / version
    assert not new_entry_dir.is_dir(), f"Module entry for version {version} exists!"

    # Add the entry
    new_entry_dir.mkdir()

    # Step 1: Set up the source json and write it
    url = f"https://github.com/resim-ai/open-core/archive/refs/tags/v{version}.zip"
    source = {
        "integrity": generate_sha256_hash(url),
        "patch_strip": 1,
        "patches": {},
        "strip_prefix": f"open-core-{version}",
        "url": url,
    }

    source_path = new_entry_dir / "source.json"

    with open(source_path, "w", encoding="utf-8") as fp:
        json.dump(source, fp)

    # Step 2: Add the MODULE.bazel
    module_url = (
        "https://raw.githubusercontent.com/"
        + f"resim-ai/open-core/refs/tags/v{version}/MODULE.bazel"
    )

    # We haven't been that good at keeping the version up to date in our MODULE.bazel, so patch it
    # when writing the MODULE.bazel
    def update_version(module_str: bytes) -> str:
        # Regex pattern to find the version inside the module block
        pattern = r'(module\s*\(\s*[^)]*?version\s*=\s*")([\d\.]+)(")'

        # Replace the old version with the new one
        return re.sub(
            pattern,
            rf"\g<1>{version}\g<3>",
            module_str.decode("utf-8"),
            flags=re.DOTALL,
        )

    # Fetch the relevant MODULE.bazel and write it to the registry, updating its version
    with (
        httpx.Client(follow_redirects=True) as client,
        open(new_entry_dir / "MODULE.bazel", "w", encoding="utf-8") as fp,
    ):
        response = client.get(module_url)
        response.raise_for_status()
        fp.write(update_version(response.content))

    # Step 3: Update the metadata:
    with open(module_path / "metadata.json", "r", encoding="utf-8") as fp:
        metadata = json.load(fp)

    metadata["versions"].append(version)
    # Shouldn't ordinarily be necessary
    metadata["versions"] = list(set(metadata["versions"]))
    metadata["versions"].sort()

    with open(module_path / "metadata.json", "w", encoding="utf-8") as fp:
        json.dump(metadata, fp, indent=2)


if __name__ == "__main__":
    main()
