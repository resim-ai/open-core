import asyncio
import hashlib
import itertools
import json
import os
from collections import defaultdict
from copy import copy
from pathlib import Path

import httpx
from tqdm.asyncio import tqdm

# fmt: off
VERSIONS = [
    "v0.1.4",  "v0.1.5",  "v0.1.6",  "v0.1.7",  "v0.1.8",  "v0.1.9",  "v0.1.10", "v0.1.11",
    "v0.1.12", "v0.1.13", "v0.1.14", "v0.1.15", "v0.1.16", "v0.1.17", "v0.1.19", "v0.1.20",
    "v0.1.21", "v0.1.22", "v0.1.23", "v0.1.24", "v0.1.26", "v0.1.27", "v0.1.28", "v0.1.29",
    "v0.1.30", "v0.1.31", "v0.2.0",  "v0.2.1",  "v0.3.0",  "v0.3.1",  "v0.3.2",  "v0.3.3",
    "v0.3.4",  "v0.3.5",  "v0.3.6",  "v0.3.7",  "v0.3.8",  "v0.3.9",  "v0.3.10", "v0.3.11",
    "v0.4.0",  "v0.5.0",  "v0.6.0",  "v0.7.0",  "v0.8.0",  "v0.9.0",  "v0.10.0", "v0.11.0",
    "v0.12.0", "v0.13.0", "v0.14.0", "v0.15.0", "v0.16.0", "v0.16.1", "v0.17.0", "v0.18.0",
    "v0.19.0", "v0.20.0", "v0.21.0", "v0.22.0", "v0.23.0", "v0.23.1", "v0.23.2", "v0.24.0",
    "v0.25.0", "v0.26.0", "v0.27.0", "v0.27.1", "v0.27.2", "v0.28.0", "v0.29.0", "v0.30.0",
    "v0.31.0", "v0.32.0", "v0.33.0", "v0.33.1", "v0.34.0", "v0.35.0", "v0.36.0", "v0.37.0", 
    "v0.38.0", "v0.39.0",
]
# fmt: on

PLATFORMS = [
    {
        "name": "linux-amd64",
        "os": "@platforms//os:linux",
        "cpu": "@platforms//cpu:x86_64",
    },
    {
        "name": "darwin-arm64",
        "os": "@platforms//os:osx",
        "cpu": "@platforms//cpu:arm64",
    },
    {
        "name": "darwin-amd64",
        "os": "@platforms//os:osx",
        "cpu": "@platforms//cpu:x86_64",
    },
]
URL_TEMPLATE = "https://github.com/resim-ai/api-client/releases/download/{}/resim-{}"

sem = asyncio.Semaphore(8)


async def compute_hash(client: httpx.AsyncClient, url: str, pbar: tqdm) -> str:
    async with sem:
        response = await client.get(url, follow_redirects=True)
        response.raise_for_status()
        sha256_hash = hashlib.sha256()
        sha256_hash.update(response.content)
        pbar.update(1)
        return sha256_hash.hexdigest()


async def main() -> None:
    workspace_dir = os.environ["BUILD_WORKSPACE_DIRECTORY"]
    assert workspace_dir, (
        "Workspace directory isn't set. Did you `bazel run` this script?"
    )
    cli_versions_bzl_path_str = os.getenv("CLI_SOURCES_BZL_PATH")
    assert cli_versions_bzl_path_str, (
        "Bzl path isn't set. Did you `bazel run` this script?"
    )
    cli_versions_bzl_path = Path(workspace_dir) / cli_versions_bzl_path_str

    all_combinations = list(itertools.product(PLATFORMS, VERSIONS))
    async with httpx.AsyncClient() as client:
        with tqdm(total=len(all_combinations)) as pbar:
            all_hashes = await asyncio.gather(
                *(
                    compute_hash(client, URL_TEMPLATE.format(v, p["name"]), pbar)
                    for (p, v) in all_combinations
                )
            )

    platforms_per_version = defaultdict(list)
    for combination, sha in zip(all_combinations, all_hashes):
        platform, version = combination
        platform = copy(platform)
        platform["sha256"] = sha
        platforms_per_version[version].append(platform)

    with open(cli_versions_bzl_path, "w", encoding="utf-8") as fp:
        fp.write(
            '"""Generated file containing the sources for all resim cli versions and platforms"""\n'
        )
        fp.write("SOURCES = {}".format(json.dumps(platforms_per_version, indent=2)))


asyncio.run(main())
