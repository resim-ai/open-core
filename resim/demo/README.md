# The SDK demos

`resim-demo` replays real test data into a fresh ReSim project so you can see
the platform working before wiring up your own. Pick one with `--demo`:

| `--demo` | What it shows |
| --- | --- |
| `navigation` (default) | A hospital navigation suite. Dense telemetry across 34 scenarios, covering every chart type ReSim ships. |
| `mujoco` | A bimanual manipulation policy in MuJoCo, one test per seed, compared across two policy builds. |

Each demo is an entry in `DEMOS` (`run.py`) plus a metrics config in `data/`.
The config is the interesting part — it is a real, working config, and the
starting point we expect you to copy:

```python
from resim.demo import config_path

print(config_path().read_text())          # the navigation demo's config
print(config_path("mujoco").read_text())  # the MuJoCo demo's config
```

## Where the replay data comes from

Each demo downloads a tarball of captured emissions and media, pinned by
sha256, and replays it into two light batches that differ only by build
version. That is what makes the A/B comparison and the trends dashboard
meaningful.

The tarballs are built from real batches by ReSim, not generated here: the
emissions are exported from runs that actually happened, downsampled so the
bundle stays small, media is transcoded down and kept only for the tests that
have it on *both* sides (an experience with video on one side reads as a broken
pair in the A/B view), and the result is checksummed and published. Adding a
demo therefore means publishing a bundle and adding a `DEMOS` entry — no
changes to `run()`.

The bundles carry no project or batch identifiers. They are published to a
public bucket, so they contain only the replayed data and the names the demo
itself shows.
