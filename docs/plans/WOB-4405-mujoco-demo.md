# A second demo, and a `--demo` flag to choose between them

Stacked on the SDK demo PR.

## Context

The demo replays one fixed dataset: a hospital navigation suite. That is a good
tour of what the platform renders, because the source data is dense telemetry
and the config can chart nearly every template ReSim ships.

It is not, however, representative of every prospect. A second demo built from
MuJoCo policy evaluations shows the platform against a different shape of data
and a different question — did this policy build get better or worse than the
last one — without pretending the two datasets look alike.

## The data

Two runs of the same 20 seeds against consecutive policy builds, so the A/B
comparison has a real difference to show:

| | baseline | candidate |
| --- | --- | --- |
| Passed / warned | 7 / 13 | 12 / 8 |
| Mean success rate | 35% | 60% |
| Mean sum reward | 264 | 325 |
| Mean evaluation time | 10.4s | 9.2s |

Seeds are identical on both sides, so every test pairs by name in the A/B view
with no positional alignment.

## What this data does and does not support

Each MuJoCo test emits four summary numbers and a recording. That is all the
replayable data there is: the rich charts in the source project come from
`metrics.binproto`, which a light batch does not reproduce, and the two
remaining emissions files carry `container_performance` and
`test_length_seconds`, whose names are reserved so a light batch cannot emit
them.

So this config covers scalar, table, bar, histogram, video and image, and does
not cover line, state_timeline or pie. There is no time series and no
categorical state in the data, and inventing one to fill the template list
would make the demo less honest, not more complete. The default demo remains
the full tour.

`config_test.py` reflects that: the checks mirroring the platform's validator
now run against every shipped demo config, while the three that assert the full
template sweep stay scoped to the default demo.

## Design

A demo is a registry entry plus a config file. `run()` is unchanged in shape —
resolve project, sync config, two batches, print links — so a third demo is
data, not code.

```python
DEMOS = {
    "navigation": Demo(project_name=…, branch=…, config_file="config.resim.yml",
                       metrics_set=…, dashboard_name=…, bundle=BundleSource(…)),
    "mujoco":     Demo(… "mujoco.resim.yml" …),
}
```

Values are spelled out per demo rather than derived from a naming convention,
so what gets downloaded is greppable and a bundle can be republished under any
name.

### Changes

- `run.py` — `Demo`, `DEMOS`, `get_demo`, and `run(demo=...)`, required rather
  than defaulted for the same reason. `config_path` and `templates_path` take a
  demo too, so a reader of either learns demos exist.
- `bundle.py` — `BundleSource` (url, sha256, cache key); `ensure` and
  `cache_dir` take one. Each demo caches under its own key, so switching demos
  does not evict the other's data.
- `__main__.py` — `--demo`, with no default. The demos are peers, so running one
  the caller did not ask for is a surprise and a bare argparse error does not
  say what is on offer; `resim-demo` with no arguments prints the demos and
  exits 0. `--project-name` and `--branch` default to the chosen demo's own
  values.
- `data/mujoco.resim.yml` — 15 metrics over three topics.

## Artifacts

Each file goes up with the log type ReSim can do the most with, following the
pattern in rerobot's `eval/resim_ingest.py`. An `.mcap` sent as
`FOXGLOVE_MCAP_LOG` opens in the viewer; the same bytes typed otherwise are only
a download, so the bundle records the type rather than leaving it to be guessed
from the filename. Per batch that is 112 attachments where there were 24.

Carrying everything was not viable. All the files both runs produced come to
271MB, downloaded on first run against a 4.8MB bundle: mcap 91.9MB, mp4 77.9MB,
`videos.zip` 77.0MB, gif 19.5MB, logs ~5MB. `videos.zip` holds the same footage
as the mp4 beside it and is dropped; the `.mcap` rides along only for jobs that
already carry media. The bundle lands at 24MB and still puts a real recording in
the viewer. An `.mcap` on all forty tests would take it to roughly 110MB.

`_log_type` falls back to inference on a type this SDK does not know, so a
bundle naming one added later degrades rather than failing the run, and a file
listed in both `artifacts` and `media` uploads once.

## Verification

- `bazel test //resim/demo:demo_test` — 136 tests, 34 new.
- New registry tests: every demo ships the config it names, names a metrics set
  and dashboard that config actually defines, and no two demos share a project,
  branch or cache key.
- The metrics-set check earned its place immediately: the MuJoCo config was
  written with `metrics_sets:`, and the platform's key is `metrics sets` with a
  space. It would have synced with no metrics sets and rendered nothing.
- Installed the wheel into a clean venv and confirmed `--demo` lists both.
- End to end against staging. The artifact run used `--data-dir` against the
  extracted bundle, so that path was proven before publishing rather than after:
  both batches SUCCEEDED and the mcaps arrived as `FOXGLOVE_MCAP_LOG`. They come
  out of the export as `MCAP_LOG`, which is what the source batch used and which
  does not reach the viewer, so the suffix decides.
- The dashboard's `Reward by Build Version` renders policy-v4 264.0 against
  policy-v5 325.05, matching the source data.

## Note on the published bundle

The manifest previously carried `source_batch_id` and the internal batch name.
The bundle is published to a public bucket, so the MuJoCo bundle was rebuilt
without them and now contains only the replayed data and the names the demo
shows.
