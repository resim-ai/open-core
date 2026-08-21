# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Export emissions, media, and the metrics config from existing ReSim batches.

A development tool, not part of the shipped ``resim.demo`` package. It captures
the raw material for the demo data bundle: for each source batch, every job's
emissions log, any image or video logs those emissions reference, and the
metrics config version the batch actually ran against.

Read-only against the API. Requires interactive browser authentication the first
time, since it uses the device code flow.

Example:
    python -m resim.demo.tools.export_source_batches \\
        --project b43bfd19-ee25-43b2-9de0-b7fd40173bbf \\
        --batch a=055b4683-53a9-47d7-944e-92819e4ba2b7 \\
        --batch b=62ea4912-db12-40e0-a30f-02287ff85297 \\
        --out ./export
"""

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Optional

import httpx

from resim.sdk.auth.device_code_client import DeviceCodeClient
from resim.sdk.bff_client.graphql import post
from resim.sdk.client import AuthenticatedClient
from resim.sdk.client.api.batches import (
    get_batch,
    list_job_logs_for_job,
    list_jobs,
)
from resim.sdk.client.models.log_type import LogType

# Log types worth keeping alongside the emissions: the media that image and
# video metrics resolve by filename.
MEDIA_LOG_TYPES = [LogType.MP4_LOG, LogType.OTHER_LOG]

MEDIA_SUFFIXES = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".mp4", ".webm"}

# A real batch emits its topic data across several EMISSIONS_LOG files, and two
# of them carry topics the platform generates itself (`container_performance`
# and `test_length_seconds`). Those names are reserved, so a light batch cannot
# emit them - skip those files rather than capturing data we can never replay.
BUILTIN_TOPIC_FILES = {
    "resource_metrics.resim.jsonl",
    "test_length_metric.resim.jsonl",
}

_CONFIG_VERSIONS = """
    query BranchConfigVersions($projectId: String!, $branchId: String!, $first: Int) {
        branchConfigVersions(projectId: $projectId, branchId: $branchId, first: $first) {
            edges {
                node {
                    id
                    insertedAt
                }
            }
        }
    }
"""

_CONFIG_CONTENT = """
    query BranchConfigContent($projectId: String!, $branchId: String!, $versionId: String) {
        branchConfigContent(projectId: $projectId, branchId: $branchId, versionId: $versionId) {
            config
            templates
            configHash
        }
    }
"""


def slugify(name: str) -> str:
    """Turn an experience name into a filesystem-safe directory name."""
    slug = re.sub(r"[^a-zA-Z0-9._-]+", "-", name).strip("-")
    return slug or "unnamed"


def fetch_jobs(
    client: AuthenticatedClient, project_id: str, batch_id: str
) -> list[Any]:
    """Page through every job in a batch."""
    jobs: list[Any] = []
    page_token: Optional[str] = None
    while True:
        kwargs: dict[str, Any] = {"client": client, "page_size": 100}
        if page_token:
            kwargs["page_token"] = page_token
        response = list_jobs.sync(project_id, batch_id, **kwargs)
        assert response is not None, f"failed to list jobs for batch {batch_id}"
        jobs.extend(response.jobs or [])
        page_token = str(response.next_page_token or "") or None
        if not page_token:
            return jobs


def fetch_logs(
    client: AuthenticatedClient,
    project_id: str,
    batch_id: str,
    job_id: str,
    log_types: list[LogType],
) -> list[Any]:
    """Page through a job's logs, restricted to the given types."""
    logs: list[Any] = []
    page_token: Optional[str] = None
    while True:
        kwargs: dict[str, Any] = {
            "client": client,
            "type_": log_types,
            "page_size": 100,
        }
        if page_token:
            kwargs["page_token"] = page_token
        response = list_job_logs_for_job.sync(project_id, batch_id, job_id, **kwargs)
        assert response is not None, f"failed to list logs for job {job_id}"
        logs.extend(response.logs or [])
        page_token = str(response.next_page_token or "") or None
        if not page_token:
            return logs


def download(url: str, destination: Path, skip_existing: bool = False) -> int:
    """Download a presigned log URL to disk, returning the byte count."""
    if skip_existing and destination.is_file() and destination.stat().st_size > 0:
        return destination.stat().st_size
    destination.parent.mkdir(parents=True, exist_ok=True)
    with httpx.stream("GET", url, follow_redirects=True, timeout=120.0) as response:
        response.raise_for_status()
        with open(destination, "wb") as f:
            for chunk in response.iter_bytes():
                f.write(chunk)
    return destination.stat().st_size


def export_config(
    client: AuthenticatedClient,
    project_id: str,
    branch_id: str,
    created_at: Any,
    out_dir: Path,
) -> None:
    """Save the metrics config version the batch ran against, plus its templates.

    Config versions are immutable and timestamped, so the version in force when
    the batch was created is the newest one inserted at or before that moment.
    """
    versions_data = post(
        client,
        _CONFIG_VERSIONS,
        "BranchConfigVersions",
        {"projectId": project_id, "branchId": branch_id, "first": 100},
    )
    edges = (versions_data.get("branchConfigVersions") or {}).get("edges") or []
    versions = [edge["node"] for edge in edges]
    versions.sort(key=lambda v: v["insertedAt"])

    version_id = None
    if created_at is not None:
        cutoff = created_at.isoformat()
        eligible = [v for v in versions if v["insertedAt"] <= cutoff]
        if eligible:
            version_id = eligible[-1]["id"]
    if version_id is None and versions:
        version_id = versions[-1]["id"]
        print(
            "    no config version predates the batch; falling back to the latest",
            file=sys.stderr,
        )

    content_data = post(
        client,
        _CONFIG_CONTENT,
        "BranchConfigContent",
        {"projectId": project_id, "branchId": branch_id, "versionId": version_id},
    )
    content = content_data.get("branchConfigContent")
    if not content:
        print("    no metrics config found for this branch", file=sys.stderr)
        return

    (out_dir / "config.resim.yml").write_text(content["config"], encoding="utf8")
    templates = content.get("templates") or {}
    for name, body in templates.items():
        path = out_dir / "templates" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(body, encoding="utf8")

    print(
        f"    config version {version_id} "
        f"({len(content['config'])} bytes, {len(templates)} template(s))"
    )


def export_batch(
    client: AuthenticatedClient,
    project_id: str,
    label: str,
    batch_id: str,
    out_root: Path,
    media_limit: int,
    media_experiences: Optional[set[str]],
    skip_existing: bool,
) -> None:
    """Export one batch's jobs, logs, and metrics config.

    Media is fetched selectively. Each job carries its own multi-megabyte camera
    clip, so pulling one per job costs hundreds of megabytes; the demo only
    needs enough of them to show the image and video metrics. Name the
    experiences to capture with ``media_experiences``, or fall back to the first
    ``media_limit`` jobs.
    """
    batch = get_batch.sync(project_id, batch_id, client=client)
    assert batch is not None, f"batch {batch_id} not found"

    out_dir = out_root / label
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"batch {label}: {batch.friendly_name} ({batch_id})")

    export_config(
        client, project_id, str(batch.branch_id), batch.creation_timestamp, out_dir
    )

    jobs = fetch_jobs(client, project_id, batch_id)
    print(f"    {len(jobs)} job(s)")

    manifest: list[dict[str, Any]] = []
    for index, job in enumerate(jobs):
        job_id = str(job.job_id)
        experience_name = str(job.experience_name)
        job_dir = out_dir / slugify(experience_name)

        emissions_logs = [
            log
            for log in fetch_logs(
                client, project_id, batch_id, job_id, [LogType.EMISSIONS_LOG]
            )
            if str(log.file_name) not in BUILTIN_TOPIC_FILES
        ]
        wants_media = (
            experience_name in media_experiences
            if media_experiences is not None
            else index < media_limit
        )
        media_logs = []
        if wants_media:
            media_logs = [
                log
                for log in fetch_logs(
                    client, project_id, batch_id, job_id, MEDIA_LOG_TYPES
                )
                if Path(str(log.file_name)).suffix.lower() in MEDIA_SUFFIXES
            ]

        files: list[str] = []
        for log in emissions_logs + media_logs:
            file_name = str(log.file_name)
            size = download(
                str(log.log_output_location), job_dir / file_name, skip_existing
            )
            files.append(file_name)
            print(f"    {experience_name}: {file_name} ({size} bytes)")

        if not emissions_logs:
            print(f"    {experience_name}: no emissions log", file=sys.stderr)

        manifest.append(
            {
                "job_id": job_id,
                "experience_name": experience_name,
                "directory": slugify(experience_name),
                "conflated_status": str(job.conflated_status),
                "emissions": [str(log.file_name) for log in emissions_logs],
                "media": [str(log.file_name) for log in media_logs],
                "files": files,
            }
        )

    (out_dir / "jobs.json").write_text(
        json.dumps(
            {
                "batch_id": batch_id,
                "batch_name": str(batch.friendly_name),
                "branch_id": str(batch.branch_id),
                "jobs": manifest,
            },
            indent=2,
        ),
        encoding="utf8",
    )


def parse_batch_arg(value: str) -> tuple[str, str]:
    """Parse a ``label=batch_id`` pair."""
    label, _, batch_id = value.partition("=")
    if not label or not batch_id:
        raise argparse.ArgumentTypeError(f"expected label=batch_id, got {value!r}")
    return label, batch_id


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", required=True, help="Project UUID.")
    parser.add_argument(
        "--batch",
        required=True,
        action="append",
        type=parse_batch_arg,
        metavar="LABEL=BATCH_ID",
        help="A batch to export, labelled (repeatable). e.g. --batch a=<uuid>",
    )
    parser.add_argument(
        "--out",
        default="export",
        type=Path,
        help="Directory to write the export into. Defaults to ./export",
    )
    parser.add_argument(
        "--media-limit",
        default=2,
        type=int,
        help=(
            "Download image and video logs for at most this many jobs per "
            "batch. Every job carries the same multi-megabyte clip, so the "
            "default of 2 is plenty."
        ),
    )
    parser.add_argument(
        "--media-experience",
        action="append",
        default=None,
        metavar="NAME",
        help=(
            "Capture media only for these experiences (repeatable). Overrides "
            "--media-limit. Use the same names for every batch so both sides "
            "of the comparison have media on the same tests."
        ),
    )
    parser.add_argument(
        "--media-experiences-file",
        default=None,
        type=Path,
        help="Like --media-experience, but one experience name per line.",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Leave already-downloaded files alone, so the export can resume.",
    )
    args = parser.parse_args()

    media_experiences: Optional[set[str]] = None
    if args.media_experience:
        media_experiences = set(args.media_experience)
    if args.media_experiences_file:
        names = [
            line.strip()
            for line in args.media_experiences_file.read_text(
                encoding="utf8"
            ).splitlines()
            if line.strip()
        ]
        media_experiences = (media_experiences or set()) | set(names)

    client = DeviceCodeClient()
    for label, batch_id in args.batch:
        export_batch(
            client,
            args.project,
            label,
            batch_id,
            args.out,
            args.media_limit,
            media_experiences,
            args.skip_existing,
        )

    print(f"\nExport written to {args.out.resolve()}")


if __name__ == "__main__":
    main()
