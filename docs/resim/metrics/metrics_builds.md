# Metrics Builds

## What are Metrics Builds

Metrics builds are code which is run *after* your test runs, taking experience and log data, and writing it to a format that the ReSim app is able to plot in app dashboard. Our current supported language for metrics is **Python**.

## How to register Metrics Builds

Just like regular test builds, you need to make a docker image and register metrics builds with the ReSim CLI before you can run them. When registered, they should appear in the ReSim app, in a separate dropdown shown when launching an experience. For more on registering metrics builds, see [Run batches with metrics](https://docs.resim.ai/run-metrics-batches/) in the ReRun docs. For the purpose of this doc, we'll assume you have such a docker image registered, and just talk about what it should do.

## Job Mode vs Batch Mode

Metrics Builds have two modes, both of which are implemented within the same single metrics build.

1. [job mode](#job-mode-computation) - in this mode, we compute metrics associated with a single job, based off output data from the job.
2. [batch mode](#batch-mode-computation) - in this mode, we compute metrics associated with a batch of jobs, based off output metrics data from `job mode`. 

Whether we are in `job mode` or `batch mode` is determined by the files present in `/tmp/resim/inputs/` when the metrics build container runs.

- If the file `/tmp/resim/inputs/batch_metrics_config.json` is present, we are in batch mode, and our build should compute batch metrics using this config.
- If the file `/tmp/resim/inputs/batch_metrics_config.json` is *not* present, then we are in job mode, and our build should compute job metrics. In this case, `/tmp/resim/inputs/` will instead contain an `experience` and `logs` directory respectively containing the input and output of the test job, based off of which we can compute our job metrics.

# Output

Regardless of the mode, your metrics build is required to write to `/tmp/resim/outputs/metrics.binproto`, in a format complying with the [JobMetrics object](https://github.com/resim-ai/open-core/blob/main/resim/metrics/proto/metrics.proto). This protobuf should validate with the [Python validator](https://github.com/resim-ai/open-core/blob/main/resim/metrics/proto/validate_metrics_proto.py) we provide.

> WARNING:
> It is highly recommended you do not interact with this proto directly, and instead use the metrics SDK documented in [Metrics Data](./metrics_data.md), [Metrics Writer](./metrics_writer.md), and [Metric Types](./metric_types.md). We reserve the right to modify this proto at any point, and do not guarantee substantial backwards compatibiliity support (such as field names remaining the same).
 
## Job Mode computation

Assuming the file `/tmp/resim/inputs/batch_metrics_config.json` is not present, we are in `job mode`! We then have two relevant folders:

1. `/tmp/resim/inputs/logs/` - this contains any log data output by the simulation job.
2. `/tmp/resim/inputs/experience/` - this contains all the experience data associated with the simulation job.

We would read these in, and as always, we'd output any metrics we want to `/tmp/resim/outputs/metrics.binproto`. For more on how to actually write these metrics see [Metrics Data](./metrics_data.md), [Metrics Writer](./metrics_writer.md), and [Metric Types](./metric_types.md).

## Batch Mode computation

> WARNING:
> The information here represents the current status of the batch metrics worker which is not final. We will improve the system by making it more generic and automated in the future

If the file `/tmp/resim/inputs/batch_metrics_config.json` is present, we are in `batch mode`! This file should look something like this:

```
{
  "authToken" : "...",
  "apiURL" : "https://api.resim.ai/v1",
  "batchID" : "7579affb-3e5b-4f02-871b-bf275aef67ee"
}
```

These three things will be used to retrieve the metrics data associated from jobs with the API. This is made easier by various tooling we provide in this repo. We then compute metrics based off this, just as we'd do for job metrics. See [Batch Metrics](./batch_metrics.md) for more on this.
