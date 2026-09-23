# A third demo: Session Evaluations

Stacked on the SDK demo and MuJoCo demo PRs.

## Context

The first two demos replay synthetic-shaped A/B suites: two builds of the same
scenarios, compared against each other. Neither shows what the platform looks
like for a customer whose workflow is not "build A vs build B" but "how did
today's field session compare to the last several" - a single robot, run
repeatedly, trended over time rather than paired off.

`session` replays four real field sessions from a legged robot (GNSS, IMU and
gait telemetry, captured 2024-11-04 through 2024-11-18) as four batches on one
branch, each batch holding exactly one test. There is no A/B pair; the
sessions trend on a shared dashboard instead.

This is also the first demo to exercise two pieces of the SDK surface the
other two never touch: a **system** (`Batch(system=...)`) and **experience
tags** on the experiences a batch's jobs create.

## What it creates

Four batches - one per session date - on a single branch, each with one test.
All four attach to the same system, `Session Evaluations`, created on first
run if it does not already exist. Every test's auto-created experience is
tagged `resim-session`, so it shows up in views that filter on that tag.

Since a batch here is one session rather than one side of a pair, the demo
prints a direct link to each session's job page (opened on its log viewer
tab) rather than a link to the batch as a whole - the job page is where the
mcap/gif content actually renders.

## Design

### Generalizing `run()` beyond two sides

The first two demos hardcode `SIDES = ("a", "b")` at module level. This demo
needs four sides, named by session date, with no "compare" link between any
pair of them. `Demo` gains:

- `sides: tuple[str, ...] = ("a", "b")` - the manifest keys this demo's bundle
  defines. Two sides get a compare link; any other count does not, since
  "compare" only means something for a pair.
- `system: Optional[str] = None` - resolved (created if missing) once, before
  any batch, then passed to every `Batch(...)`.
- `experience_tag: Optional[str] = None` - resolved (created if missing) once;
  every test's auto-created experience is then tagged with it.

`run()` loops over `chosen.sides` instead of the module constant. `_urls` and
`_report` generalize the same way: `_urls` links a batch whose manifest lists
exactly one job straight to that job (`links.job_url`, opened on
`defaultTab=0`) instead of to the batch; `_report` keeps the existing
"baseline/candidate/compare" framing for exactly two sides and falls back to
one line per side otherwise.

### System and experience tags

Neither has an SDK helper today. `Batch(system=...)` only resolves an
existing system by name and raises if none matches - there is no
`resolve_system` in the SDK, and none for experience tags either. Added to
`resim/demo/run.py` (not to `Batch`/`Test` themselves, since this is demo
orchestration, not something every SDK caller needs):

- `resolve_system` - list-by-name, create if missing. `createSystemInput`
  requires build/metrics-build compute fields that a light batch never
  actually uses (no container runs under it); these get small fixed values,
  documented as unused.
- `resolve_experience_tag` - list-by-name (paginated), create if missing.
- `tag_experience` - `add_experience_tag_to_experience`, treating a 409
  (already tagged) as success rather than an error, since re-running the demo
  against the same project hits this on every pass after the first.

The experience to tag is not created by the demo at all: `create_job_for_batch`
auto-creates or matches an experience for every job whether or not the caller
names one, and the job response carries its id. `Test` gains two read-only
properties, `job_id` and `experience_id`, exposing the underlying job
response's fields - previously accessible only by reaching into `Test._test`.

### Config

`resim/demo/data/session.resim.yml`. Thirteen topics matching the session
export's real fields exactly (the SDK's `Emitter` validates emitted payloads
field-for-field against the schema, so this is not approximate): per-mission
summaries, motion phases, movement-transition snapshots (a gif per
start/stop), pose/IMU/GNSS/battery/command time series, mission-boundary
events, a session summary, and a record of the raw source file the session's
mcap snippets were cut from.

The metrics are the ones the internal `grandtour-sessions` project runs on
the same four sessions: 39 test-level metrics, all on system templates (bar,
line, scalar, table, histogram, pie, image), grouped into one `Session
Metrics` set. They cover per-mission speed, distance, path efficiency and
stop behaviour, odometry-vs-GNSS agreement, altitude, tracking error, joint
effort, IMU shock, battery, motion phases and the event breakdown. Keeping
them identical to the internal project means the demo shows what the team
actually looks at for this data.

There are no batch or dashboard metrics: with one test per batch, a batch
view would repeat the test view. `config_test.py` therefore exempts the
session demo from the rule that every config carries batch metrics, the
same way it already exempts configs without a dashboard.

`TemplateCoverageTest` checks that every shipped template in `data/templates/`
is used by *some* registered demo. The session demo uses only system
templates, so it ships none of its own.

## The data bundle

Downloaded via the `resim` CLI (`logs download`, per test - batch-level
download without `--test-id` returns nothing) from four real batches in an
internal `grandtour-sessions` project, one per session date: emissions,
mcap snippets and gif clips, unmodified. Built into a `manifest.json` in the
same shape the first two demos use - `batches.<session>.jobs[0]` - so
`bundle.py` needed no changes.

Uncompressed this is roughly three times the size of the MuJoCo bundle: full
mcap snippets and gifs for four real sessions, not downsampled or transcoded.
Downsampling media the way the MuJoCo bundle does is a reasonable follow-up
if the published size becomes a problem, but is out of scope here.

## Tests

- `resim/sdk/test_test.py` - `job_id` and `experience_id` properties.
- `resim/demo/run_test.py`:
  - `ResolveSystemTest`, `ResolveExperienceTagTest`, `TagExperienceTest` -
    the three new helpers, unit-tested against a stubbed client the same way
    `ResolveProjectTest` already covers `resolve_project`.
  - `UrlsTest` - job-vs-batch link selection.
  - `SessionOrchestrationTest` - system resolution, experience tagging, and
    job-level links, exercised against navigation's real config with `sides`/
    `system`/`experience_tag` overridden rather than duplicating a second
    full config's worth of fixture emissions.
- `resim/demo/config_test.py` - `SessionConfigTest` (the same rules every
  config gets) and `TemplateCoverageTest` (replacing the assumption a single
  config owns every shipped template).
- `resim/demo/links_test.py` - `job_url`.

## Verification

1. `bazel test //resim/demo:demo_test //resim/sdk:sdk_test` - all pass.
2. End to end against staging: TODO once the bundle is published.

## Known follow-ups

- The published bundle's sha256/URL are provisional pending an actual upload
  to `s3://resim-public-assets/sdk-demo/` - the same manual step the other
  two demos' bundles need.
- Media is not downsampled or transcoded, unlike the MuJoCo bundle. Worth
  revisiting if the published size is a problem.
