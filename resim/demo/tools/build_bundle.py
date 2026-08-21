# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Turn a source-batch export into the demo's replay bundle.

A development tool, not part of the shipped ``resim.demo`` package. Takes the
directory written by ``export_source_batches`` and produces the single tarball
the demo downloads at run time:

    resim-sdk-demo-data-v1.tar.gz
      manifest.json
      a/<experience>/emissions.resim.jsonl
      a/<experience>/camera.mp4
      a/<experience>/camera_frame.jpg
      b/...

Three things happen on the way through:

- **Filtering.** A real batch emits topics the platform generates itself, and
  topics the demo's config does not declare. Only declared topics survive, so
  the data and the config cannot drift apart.
- **Downsampling.** The source runs are three minutes of dense telemetry. The
  charts look the same at a fraction of the samples, and the demo uploads a
  fraction of the bytes.
- **Media.** Each source clip is 5-7MB of 1152x720. They are transcoded to a
  short, small clip, and a still frame is pulled out for the image metric.

Example:
    python -m resim.demo.tools.build_bundle \\
        --export ./export --out ./resim-sdk-demo-data-v1.tar.gz
"""

import argparse
import gzip
import hashlib
import json
import shutil
import subprocess
import sys
import tarfile
import tempfile
from collections import defaultdict
from pathlib import Path
from typing import Any, Optional

import yaml

from resim.sdk.metrics.emissions import Emitter, ReSimValidationError

# Emissions files carrying topics the platform generates for itself. Their
# names are reserved, so a light batch cannot re-emit them.
BUILTIN_TOPIC_FILES = {
    "resource_metrics.resim.jsonl",
    "test_length_metric.resim.jsonl",
}

EMISSIONS_NAME = "emissions.resim.jsonl"
VIDEO_NAME = "camera.mp4"
FRAME_NAME = "camera_frame.jpg"

# Rows kept per topic per job, per series within that topic. Dense telemetry is
# sampled evenly so the shape of each chart survives; anything not listed is
# kept in full.
ROW_CAPS = {
    "odom_linear_velocity": 250,
    "goal_distance": 120,
    "nearest_human_distance": 120,
    "pose_difference": 200,
    "localization_uncertainty": 200,
    "covariance_accuracy": 200,
}

# How the two sides present in the app. Same branch, different build version:
# that is what the A/B comparison highlights, and what lets one dashboard trend
# across both.
SIDES = {
    "a": {
        "name": "Nav stack v2 (baseline)",
        "version": "nav-v2.0.0",
        "description": "Baseline run of the hospital navigation suite.",
    },
    "b": {
        "name": "Nav stack v3 (candidate)",
        "version": "nav-v3.0.0",
        "description": "Candidate run of the same suite, for comparison.",
    },
}

VIDEO_START_SECONDS = 20
VIDEO_DURATION_SECONDS = 20
VIDEO_WIDTH = 640
VIDEO_CRF = 32
FRAME_AT_SECONDS = 45
FRAME_WIDTH = 800


class BuildError(RuntimeError):
    """The bundle could not be built from the given export."""


def ffmpeg() -> str:
    """Locate an ffmpeg binary, preferring the pip-installed one."""
    try:
        import imageio_ffmpeg

        return str(imageio_ffmpeg.get_ffmpeg_exe())
    except ImportError:
        pass
    found = shutil.which("ffmpeg")
    if found:
        return found
    raise BuildError(
        "no ffmpeg found. Install it, or `pip install imageio-ffmpeg`, so the "
        "camera clips can be transcoded."
    )


def validate(emitter: Emitter, records: list[dict[str, Any]], where: str) -> None:
    """Check records against the config the demo ships, before publishing them.

    The demo replays through the typed ``Test`` methods, which validate as they
    go, so anything that fails here would fail on someone's first run. Catch it
    while the bundle is still being built.
    """
    for number, record in enumerate(records, start=1):
        topic = record["$metadata"]["topic"]
        try:
            if record["$metadata"].get("event"):
                emitter._validate_event_topic(topic)
            emitter._validate_data(topic, record["$data"])
        except ReSimValidationError as e:
            raise BuildError(f"{where} emission {number} ({topic}): {e}") from e


def declared_topics(config_path: Path) -> set[str]:
    """Topic names the demo's metrics config declares."""
    config = yaml.safe_load(config_path.read_text(encoding="utf8"))
    topics = (config or {}).get("topics") or {}
    if not topics:
        raise BuildError(f"no topics declared in {config_path}")
    return set(topics)


def series_key(payload: dict[str, Any]) -> tuple:
    """Identify the series a row belongs to, by its non-numeric fields.

    ``goal_distance`` carries one series per goal and ``nearest_human_distance``
    one per person. Sampling has to happen inside each series, or thinning the
    data would drop whole goals rather than thinning each line.
    """
    return tuple(
        sorted((key, value) for key, value in payload.items() if isinstance(value, str))
    )


def downsample(records: list[dict[str, Any]], cap: int) -> list[dict[str, Any]]:
    """Keep at most ``cap`` evenly spaced records, preserving the first and last."""
    if len(records) <= cap:
        return records
    if cap <= 2:
        return records[:cap]
    step = (len(records) - 1) / (cap - 1)
    picked = [records[round(i * step)] for i in range(cap)]
    return picked


def read_emissions(job_dir: Path, topics: set[str]) -> list[dict[str, Any]]:
    """Read every emission for a job, keeping only declared topics.

    A real batch spreads its topic data over several emissions files, so all of
    them are read and merged.
    """
    records: list[dict[str, Any]] = []
    for path in sorted(job_dir.glob("*.resim.jsonl")):
        if path.name in BUILTIN_TOPIC_FILES:
            continue
        for number, line in enumerate(
            path.read_text(encoding="utf8").splitlines(), start=1
        ):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
                topic = record["$metadata"]["topic"]
            except (json.JSONDecodeError, KeyError, TypeError) as e:
                raise BuildError(f"malformed emission at {path}:{number}: {e}") from e
            if topic in topics:
                records.append(record)
    return records


def thin(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Apply ROW_CAPS per topic and series, keeping the original ordering."""
    grouped: dict[tuple, list[dict[str, Any]]] = defaultdict(list)
    order: list[tuple] = []
    for record in records:
        topic = record["$metadata"]["topic"]
        key = (topic, series_key(record.get("$data") or {}))
        if key not in grouped:
            order.append(key)
        grouped[key].append(record)

    kept: list[dict[str, Any]] = []
    for key in order:
        topic = key[0]
        cap = ROW_CAPS.get(topic)
        kept.extend(grouped[key] if cap is None else downsample(grouped[key], cap))

    kept.sort(key=lambda r: (r["$metadata"].get("timestamp") or 0))
    return kept


def transcode(binary: str, source: Path, destination: Path) -> None:
    """Write a short, small version of a camera clip."""
    _run(
        [
            binary,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-ss",
            str(VIDEO_START_SECONDS),
            "-t",
            str(VIDEO_DURATION_SECONDS),
            "-i",
            str(source),
            "-vf",
            f"scale={VIDEO_WIDTH}:-2",
            "-c:v",
            "libx264",
            "-crf",
            str(VIDEO_CRF),
            "-preset",
            "slow",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            "-an",
            str(destination),
        ]
    )


def extract_frame(binary: str, source: Path, destination: Path) -> None:
    """Pull a single frame out of a camera clip, for the image metric."""
    _run(
        [
            binary,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-ss",
            str(FRAME_AT_SECONDS),
            "-i",
            str(source),
            "-frames:v",
            "1",
            "-vf",
            f"scale={FRAME_WIDTH}:-2",
            "-q:v",
            "4",
            str(destination),
        ]
    )


def _run(command: list[str]) -> None:
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        raise BuildError(
            f"{Path(command[0]).name} failed: {result.stderr.strip() or result.returncode}"
        )


def camera_name(records: list[dict[str, Any]]) -> str:
    """Reuse the camera name the source data reported, so the frame matches."""
    for record in records:
        if record["$metadata"]["topic"] == "camera_video":
            name = (record.get("$data") or {}).get("camera_name")
            if isinstance(name, str):
                return name
    return "camera"


def media_experiences(
    export_root: Path, sides: list[str], experiences: list[str]
) -> set[str]:
    """Experiences with a clip on *every* side.

    The A/B view puts the same metric from both runs side by side, so an
    experience with video on one side only reads as a broken pair. Keeping media
    for the intersection makes that impossible.
    """
    per_side = [
        {
            experience
            for experience in experiences
            if (
                export_root
                / side
                / _directory(export_root, side, experience)
                / VIDEO_NAME
            ).is_file()
        }
        for side in sides
    ]
    shared = set.intersection(*per_side) if per_side else set()
    dropped = set.union(*per_side) - shared if per_side else set()
    if dropped:
        print(
            f"  {len(dropped)} experience(s) have media on only one side; "
            "dropping their media to keep the pairs symmetrical",
            file=sys.stderr,
        )
    return shared


def _directory(export_root: Path, side: str, experience: str) -> str:
    index = json.loads((export_root / side / "jobs.json").read_text(encoding="utf8"))
    for job in index["jobs"]:
        if job["experience_name"] == experience:
            return str(job["directory"])
    raise BuildError(f"{side} has no job for experience {experience!r}")


def build_side(
    side: str,
    export_root: Path,
    staging: Path,
    topics: set[str],
    experiences: list[str],
    with_media: set[str],
    binary: Optional[str],
    emitter: Emitter,
) -> dict[str, Any]:
    """Build one side of the comparison, returning its manifest entry."""
    source = export_root / side
    jobs_index = json.loads((source / "jobs.json").read_text(encoding="utf8"))
    by_experience = {job["experience_name"]: job for job in jobs_index["jobs"]}

    entries: list[dict[str, Any]] = []
    for experience in experiences:
        job = by_experience[experience]
        job_dir = source / job["directory"]
        out_dir = staging / side / job["directory"]
        out_dir.mkdir(parents=True, exist_ok=True)

        records = read_emissions(job_dir, topics)
        if not records:
            raise BuildError(f"{side}/{experience} has no usable emissions")
        media: list[str] = []

        clip = job_dir / VIDEO_NAME
        if experience in with_media and clip.is_file():
            assert binary is not None
            transcode(binary, clip, out_dir / VIDEO_NAME)
            extract_frame(binary, clip, out_dir / FRAME_NAME)
            media = [VIDEO_NAME, FRAME_NAME]
            # The source data has no image topic, so the still frame gets one
            # emission of its own alongside the clip it came from.
            records.append(
                {
                    "$metadata": {"topic": "camera_frame"},
                    "$data": {
                        "camera_name": camera_name(records),
                        "filename": FRAME_NAME,
                    },
                }
            )
        else:
            # Without a clip there is nothing for the video metric to resolve,
            # so drop the reference rather than leaving a dangling filename.
            records = [r for r in records if r["$metadata"]["topic"] != "camera_video"]

        kept = thin(records)
        validate(emitter, kept, f"{side}/{experience}")
        (out_dir / EMISSIONS_NAME).write_text(
            "".join(json.dumps(record) + "\n" for record in kept), encoding="utf8"
        )

        entries.append(
            {
                "experience_name": experience,
                "directory": f"{side}/{job['directory']}",
                "emissions": EMISSIONS_NAME,
                "media": media,
                "emission_count": len(kept),
                "source_status": job.get("conflated_status"),
            }
        )
        print(
            f"  {side}/{experience}: {len(records)} -> {len(kept)} emissions"
            f"{', media' if media else ''}"
        )

    return {
        **SIDES[side],
        "source_batch_id": jobs_index["batch_id"],
        "source_batch_name": jobs_index["batch_name"],
        "jobs": entries,
    }


def _replayable(job_dir: Path, topics: set[str]) -> bool:
    """Whether a captured job has data the demo could actually replay.

    A source job that errored leaves either no logs at all, sometimes not even
    a directory, or logs holding only topics the config does not declare.
    """
    if not job_dir.is_dir():
        return False
    return bool(read_emissions(job_dir, topics))


def paired_experiences(
    export_root: Path, sides: list[str], topics: set[str]
) -> list[str]:
    """Experiences every side can replay, which is what pairs the tests.

    The A/B comparison matches tests by experience name, so an experience
    present on one side only shows up as unique rather than as a pair. An
    experience whose source job emitted nothing is dropped from every side for
    the same reason: half a pair is worse than none.
    """
    per_side = {}
    for side in sides:
        index = json.loads(
            (export_root / side / "jobs.json").read_text(encoding="utf8")
        )
        per_side[side] = {
            job["experience_name"]
            for job in index["jobs"]
            if _replayable(export_root / side / job["directory"], topics)
        }

    shared = set.intersection(*per_side.values())
    dropped = set.union(*per_side.values()) - shared
    if dropped:
        print(
            f"  dropping {len(dropped)} experience(s) that not every side can "
            f"replay: {', '.join(sorted(dropped))}",
            file=sys.stderr,
        )
    if len(shared) < 10:
        raise BuildError(
            f"only {len(shared)} experience(s) are shared across all sides; "
            "the comparison needs more than that to be worth looking at"
        )
    return sorted(shared)


def write_tarball(staging: Path, destination: Path) -> None:
    """Write the bundle deterministically.

    Same inputs, same bytes, same checksum. Otherwise a rebuild of identical
    content produces a new digest and looks like the data changed - so member
    order and metadata are normalised, and gzip's timestamp is zeroed.
    """

    def normalise(info: tarfile.TarInfo) -> tarfile.TarInfo:
        info.uid = info.gid = 0
        info.uname = info.gname = ""
        info.mtime = 0
        info.mode = 0o644
        return info

    with open(destination, "wb") as raw:
        # filename="" keeps gzip from recording the output path in its header,
        # which would otherwise make the digest depend on where it was written.
        with gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as gz:
            with tarfile.open(fileobj=gz, mode="w", format=tarfile.GNU_FORMAT) as tar:
                for path in sorted(staging.rglob("*")):
                    if path.is_file():
                        tar.add(
                            path,
                            arcname=str(path.relative_to(staging)),
                            filter=normalise,
                        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--export",
        required=True,
        type=Path,
        help="Directory written by export_source_batches.",
    )
    parser.add_argument(
        "--out",
        required=True,
        type=Path,
        help="Tarball to write.",
    )
    parser.add_argument(
        "--config",
        default=Path(__file__).resolve().parent.parent / "data" / "config.resim.yml",
        type=Path,
        help="Metrics config whose topics decide what is kept.",
    )
    parser.add_argument(
        "--max-tests",
        default=None,
        type=int,
        help="Keep at most this many experiences per side. Defaults to all.",
    )
    args = parser.parse_args()

    try:
        topics = declared_topics(args.config)
        experiences = paired_experiences(args.export, list(SIDES), topics)
        if args.max_tests is not None:
            experiences = experiences[: args.max_tests]
        with_media = media_experiences(args.export, list(SIDES), experiences)
        binary = ffmpeg() if with_media else None

        print(
            f"{len(experiences)} experience(s) per side, {len(topics)} topic(s), "
            f"{len(with_media)} with media"
        )

        with tempfile.TemporaryDirectory() as temp:
            staging = Path(temp)
            # Validation only, so the emitter's own output is thrown away.
            emitter = Emitter(
                config_path=args.config, output_path=staging / "validation.jsonl"
            )
            manifest = {
                "version": 1,
                "batches": {
                    side: build_side(
                        side,
                        args.export,
                        staging,
                        topics,
                        experiences,
                        with_media,
                        binary,
                        emitter,
                    )
                    for side in SIDES
                },
            }
            emitter.close()
            (staging / "validation.jsonl").unlink(missing_ok=True)
            (staging / "manifest.json").write_text(
                json.dumps(manifest, indent=2), encoding="utf8"
            )

            args.out.parent.mkdir(parents=True, exist_ok=True)
            write_tarball(staging, args.out)
    except BuildError as e:
        print(f"build_bundle: {e}", file=sys.stderr)
        return 1

    digest = hashlib.sha256(args.out.read_bytes()).hexdigest()
    print(f"\nWrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")
    print(f"sha256 {digest}")
    print("\nSet BUNDLE_SHA256 in resim/demo/bundle.py to that value, then upload:")
    print(f"  aws s3 cp {args.out} s3://resim-public-assets/sdk-demo/{args.out.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
