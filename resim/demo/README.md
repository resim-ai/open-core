# The SDK demo

`resim-demo` replays real test data into a fresh ReSim project so you can see
the platform working before wiring up your own tests: two comparable batches of
tests on one branch, and a dashboard that trends across them.

The interesting part is `data/config.resim.yml`. It is a real, working metrics
config — not a toy — and it is the starting point we expect you to copy:

```python
from resim.demo import config_path

print(config_path().read_text())
```

## Where the replay data comes from

The demo downloads a tarball of captured emissions and media, pinned by sha256,
and replays it into two light batches that differ only by build version. That
difference is what makes the A/B comparison and the trends dashboard
meaningful.

The tarball is built from real batches by ReSim rather than generated here. The
emissions come from runs that actually happened, downsampled so the bundle
stays small; media is transcoded down and kept only for tests that have it on
*both* sides, since an experience with video on one side reads as a broken pair
in the A/B view; and the result is checksummed and published.

The bundle carries no project or batch identifiers. It is published to a public
bucket, so it holds only the replayed data and the names the demo itself shows.
