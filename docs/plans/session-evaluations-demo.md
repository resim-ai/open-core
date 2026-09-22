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

Test-level metrics cover line (both the system template and a custom Liquid
one - `custom_line.liquid`, carried over from an internal config used to
exercise the config validator, generalized to any query aliasing its columns
to `timestamp`/`speeds`/`group_name`) and table charts; batch-level metrics
cover table and scalar, including a status check that warns when a session's
mean GNSS coverage drops below half; dashboard-level metrics chart distance
and mean speed per session as bars grouped by `build_version`, which the demo
sets to the session date.

Adding `custom_line.liquid` to the shared `data/templates/` directory exposed
a real gap in `config_test.py`: `NavigationCoverageTest.test_every_shipped_template_is_used`
assumed every shipped template belonged to the flagship demo, which broke the
moment a second demo brought its own template. Replaced with
`TemplateCoverageTest`, which checks that every shipped template is used by
*some* registered demo rather than by `config.resim.yml` specifically.

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
