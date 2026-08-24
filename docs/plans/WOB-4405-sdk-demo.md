# WOB-4405 — Self-serve SDK demo: `resim.demo.run()`

Ticket: [WOB-4405](https://linear.app/resim/issue/WOB-4405/self-serve-sdk-demo-resimdemorun)
(Wobblies, project *Self Serve App Access M1*).

Two repos, two PRs, same ticket:

- **`resim-ai/open-core`** — the demo package. This plan.
- **`resim-ai/docs`** — the customer-facing tutorial, at `docs/tutorials/sdk-demo.md`.
  Lands second, because it cites the installed package and screenshots of a real run.

## Context

Someone who has just run `pip install resim-open-core` has nothing to look at. The
existing starting points are a two-test script (`resim/examples/external_batches/`,
not wired into `resim/examples/BUILD`, and not shipped in the wheel) and the
`resim-ai/resim-sdk-example` repo, whose config defines two metrics over one topic.
Neither shows metric variety, pass/fail status, A/B comparison between two runs, or
trends across runs.

`resim.demo.run()` populates a project end to end and hands back four links: batch A,
batch B, the A/B comparison, and a trends dashboard. Nothing to configure, no Docker,
no build to register.

## What it creates

One project, one branch, two light batches of 34 tests each. Both batches run the
same 36 experience names, which is what pairs their tests in the A/B view; they
differ by build version (`nav-v2.0.0` vs `nav-v3.0.0`), which is what the comparison
cards highlight and what the dashboard trends along. Both live on one branch, because
a dashboard is scoped to a single branch.

The data is a replay of two real batches from the hospital navigation demo project
(`055b4683…` and `62ea4912…`, "Hospital Demo v2" and "v3"). Those two batches happen
to be an ideal A/B: identical 36-experience sets, with statuses differing on 9 of
them, so all four groups of the comparison's Tests tab populate from real results
rather than a contrived story. Two of the 36 errored in the source and emitted
nothing, so they are dropped from both sides, leaving 34 pairs and 8 differing.
Pass/fail is not copied across; it is recomputed by the config's status checks
against the replayed data.

## Layout

Shipped, under `resim/demo/`:

| File | Responsibility |
| --- | --- |
| `__init__.py` | Exports `run`, `DemoResult`, `DemoDataError`. |
| `__main__.py` | `python -m resim.demo` and the `resim-demo` console script. |
| `run.py` | Orchestration: resolve project, run both batches, print links. |
| `bundle.py` | Download, verify, safely extract and cache the replay data. |
| `links.py` | Build the four app URLs from the client's API base URL. |
| `data/config.resim.yml` | 13 topics, 21 metrics, 2 metrics sets, 1 dashboard. |
| `data/templates/*.liquid` | The two custom templates the config references. |

The scripts that produce the published tarball — one to capture a source batch's
emissions, media and config version, one to turn that capture into the bundle —
are deliberately not in this repo. They take internal project and batch IDs as
input and this repo is public, so `resim/demo/README.md` describes what they do
and the scripts themselves live with the rest of our internal tooling.

Supporting changes elsewhere:

- `resim/sdk/bff_client/graphql.py` — `bff_url` and `post`, extracted from the private
  `_get_bff_url` in `metrics.py`. `sync_config` now goes through it.
- `resim/sdk/bff_client/dashboards.py` — `find_dashboard_id`, so the demo links
  straight to the dashboard rather than to the dashboards list. Note that
  `DashboardConnection` exposes `nodes`, not `edges`.
- `Batch.branch_id` is now public, since the dashboard lookup is branch-scoped.

## The data bundle

Too large for the wheel, so it is one 6.9MB tarball in a public bucket, downloaded
and cached under `${XDG_CACHE_HOME:-~/.cache}/resim/sdk-demo/<version>/` on first
run. There is no local-generation fallback: if the download fails, the demo exits
with the URL and the reason. The tarball is byte-reproducible — same inputs, same
digest — so a rebuild cannot look like a data change.

The bundle build does four things to the export:

- **Filters** to the topics the shipped config declares. A real batch also emits
  `container_performance` and `test_length_seconds`, whose names are reserved, so a
  light batch cannot re-emit them. Filtering against the config means data and config
  cannot drift apart.
- **Merges** each job's several emissions files. The source spreads its topics over
  `emissions.resim.jsonl` and `metrics.resim.jsonl`.
- **Downsamples** per topic *and per series* — `goal_distance` carries one series per
  goal, so a flat cap would drop whole goals rather than thinning each line.
- **Transcodes media.** Each source clip is 5–7MB of 1152x720; they become ~250KB
  clips, and a frame is extracted as a ~24KB JPEG. Media is kept for 12 of the 36
  experiences — the 9 whose status differs plus 3 stable ones — the same 12 on both
  sides, so the media metrics appear on both halves of those pairs. The source data
  has no image topic, so the extracted frame gets a `camera_frame` topic emission of
  its own; a topic may not carry both an image and a video column.

Publishing is manual: build the bundle, set `BUNDLE_SHA256` in `bundle.py` to the
printed digest, and upload to `s3://resim-public-assets/sdk-demo/`.

## Metrics coverage

The config exercises every system template the platform ships — `line`, `bar`,
`table`, `scalar`, `image`, `video`, `state_timeline`, `histogram`, `pie` — plus both
custom Liquid templates, at all three metric levels (13 test, 5 batch, 3 dashboard),
with status checks that drive pass/fail and an event topic that populates the Events
tab. The dashboard metrics group by `build_version`, so they compare versions of the
stack rather than snapshotting one run: `Localization Error by Build Version` plots
mean localizer error per version, and `Pass Rate by Build Version` the share of tests
each version passed, with `link_path` click-through to a batch that version ran in.
Re-running the demo folds new runs into the same points rather than multiplying them,
which is what you want from a dashboard pinned to a branch.

## Tests

All offline. `//resim/demo:demo_test` runs 66 of them; the new
`//resim/sdk/bff_client:dashboards_test` adds 6.

- `config_test.py` mirrors the rules the BFF's config validator enforces, so a bad
  config fails in CI rather than on someone's first run: metrics sets and dashboards
  reference things that exist, templates are known and shipped, no topic shadows a
  builtin table or mixes image and video columns, every table a query reads is
  declared (accounting for CTEs), status checks take exactly one threshold parameter.
  It also asserts the demo still covers every template, which is the point of it.
- `bundle_test.py` covers checksum mismatch, path traversal and symlink members, warm
  and cold cache, and that a download failure names the URL.
- `run_test.py` covers orchestration against a stubbed API: both batches on one
  branch with the same experience names and different versions, the four printed
  URLs, the dashboard fallback, and cleanup of the emissions files `Test` leaves in
  the working directory. Its fake test object is a real `Emitter` built from the
  shipped config, so every replayed emission is validated against it.

## Verification

1. `bazel test //resim/demo:demo_test //resim/sdk:sdk_test
   //resim/sdk/bff_client:dashboards_test //resim/sdk/bff_client:metrics_test` — all
   four pass. 72 of those tests are new (66 in `demo_test`, 6 in `dashboards_test`).
2. `bazel build //pkg:sdk_wheel`, then install the wheel into a clean venv. Confirmed
   `entry_points.txt` carries `resim-demo = resim.demo.__main__:main`, the package
   ships `data/config.resim.yml` and both `.liquid` files, tests are excluded,
   `from resim.demo import run` works, and `resim-demo --help` resolves.
3. End to end against staging (`RESIM_API_URL=https://api.resim.io/v1` with the dev
   Auth0 tenant). Confirmed across two runs:
   - Project, branch, 68 tests across two batches in about four minutes, and no
     files left in the working directory.
   - **13 test metrics** covering every system template — LINE, HISTOGRAM, PIE,
     STATE_TIMELINE, TABLE, SCALAR, VIDEO, IMAGE — plus both CUSTOM Liquid
     templates, with statuses spanning PASSED, FAIL_WARN and FAIL_BLOCK.
   - **5 batch metrics** rendering on a batch that finished clean: BAR, CUSTOM,
     TABLE, BAR, SCALAR.
   - **3 dashboard metrics** rendering, and the dashboard itself came back
     `source: CONFIG`, `autoRefresh: true`, `dayRange: 30`. (They grouped by
     experience at the time; they now group by `build_version`.)
   - Status checks reproduce the source batch's verdicts from replayed data:
     `{PASSED: 16, WARNING: 9, BLOCKER: 9}` against the source's
     `{PASSED: 17, WARNING: 9, BLOCKER: 9}`, the difference being the two dropped
     experiences.
4. Re-run against the same project: branch and dashboard reused (same dashboard ID),
   two fresh batches, another point per experience on the trend.
5. Cold-cache failure path: with nothing published at `BUNDLE_URL`, the demo names
   the URL and the HTTP status, exits 1, and fails before prompting for login.

Staging errored a large share of jobs during the metrics phase, with every failing
chart reporting `causedBy: RESIM_PLATFORM` and the redacted `INTERNAL_RESIM_ERROR`,
no error or container logs, and the rate varying run to run — one batch finished
completely clean on the second attempt. Batch-level metrics do not run once a batch
ends in `ERROR`, which is why those charts were confirmed on the clean batch.

## Known follow-ups

- `Test` writes `emissions_<job_id>.resim.jsonl` to the working directory and leaves
  it. The demo deletes its own; fixing the SDK to use a temp directory would change
  behaviour others may depend on, so it is out of scope here.
- A public `fetch_config`, the read counterpart to `sync_config`, would let the export
  tool use `bff_client` rather than carrying its own two GraphQL queries.
- Tests upload sequentially, at roughly 3.5s each, which is where the tutorial's
  "about 5 minutes" comes from. A thread pool would cut it to about a minute; left
  sequential for now so a failure is easy to attribute.
