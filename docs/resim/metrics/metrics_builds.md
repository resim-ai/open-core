# Metrics Builds

## What are Metrics Builds?

Metrics builds are code which is run *after* your test runs, taking experience and log data, and writing it to a format that the ReSim app is able to plot in app dashboard. Our current supported language for metrics is **Python**.

## How to Register Metrics Builds

Just like regular system builds, you need to make a docker image and register metrics builds with the ReSim CLI before you can run them. When registered, they should appear in the ReSim app when creating test suites or running ad hoc tests. For more on registering metrics builds, see [Run batches with metrics](https://docs.resim.ai/setup/metrics-builds/) in the ReRun docs. For the purpose of this doc, we'll assume you have such a docker image registered, and just talk about what it should do.

## Test Mode vs Batch Mode

Metrics Builds have three modes, two of which *must implemented within the same metrics build*.

1. [test mode](#test-mode-computation) - in this mode, we compute metrics associated with a single test, based off output data from the test.
2. [batch mode](#batch-mode-computation) - in this mode, we compute metrics associated with a batch of tests, based off output metrics data from `test mode`. 

Whether we are in `test mode` or `batch mode` is determined by the files present in `/tmp/resim/inputs/` when the metrics build container runs.

- If the file `/tmp/resim/inputs/batch_metrics_config.json` is present, we are in batch mode, and our build should compute batch metrics using this config.
- If the file `/tmp/resim/inputs/batch_metrics_config.json` is *not* present, then we are in test mode, and our build should compute test metrics. In this case, `/tmp/resim/inputs/` will instead contain an `experience` and `logs` directory respectively containing the input and output of the test which we can use to compute our metrics.

## Test Suite Report Mode

The third mode that a metrics build can operate in exists to compute **test suite reports**. A test suite report is a longitudinal comparison of a given [test suite](https://docs.resim.ai/setup/test-suites) over time. A report is expected to compute how performance of a given branch of the system has changed over time. For more information, please see the main [reports documentation](https://docs.resim.ai/setup/reports).

For a metrics build to determine that it is in `report mode`, it must check that a file called `report_config.json` exists in the `/tmp/resim/inputs` directory. If this file is present, we are in report mode and we should compute reports.

# Output

Regardless of the mode, in order to have metrics show up your metrics build is required to write to `/tmp/resim/outputs/metrics.binproto`, in a format complying with the [JobMetrics object](https://github.com/resim-ai/open-core/blob/main/resim/metrics/proto/metrics.proto). This protobuf should validate with the [Python validator](https://github.com/resim-ai/open-core/blob/main/resim/metrics/proto/validate_metrics_proto.py) we provide.

> WARNING:
> It is highly recommended you do not interact with this proto directly, and instead use the metrics SDK documented in [Metrics Data](./metrics_data.md), [Metrics Writer](./metrics_writer.md), [Metric Types](./metric_types.md), and [Events](./events.md). We reserve the right to modify this proto at any point, and do not guarantee substantial backwards compatibiliity support (such as field names remaining the same).
 
## Test Mode Computation

Assuming the file `/tmp/resim/inputs/batch_metrics_config.json` is not present, we are in `test mode`! We then have two relevant directories:

1. `/tmp/resim/inputs/logs/` - this contains any log data output by the simulation.
2. `/tmp/resim/inputs/experience/` - this contains all the experience data associated with the simulation.

We would read these in, and as always, we'd output any metrics we want to `/tmp/resim/outputs/metrics.binproto`. For more on how to actually write these metrics see [Metrics Data](./metrics_data.md), [Metrics Writer](./metrics_writer.md), [Metric Types](./metric_types.md), and [Events](./events.md).

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

These four things will be used to retrieve the metrics data associated from tests with the API. This is made easier by various tooling we provide in this repo. We then compute metrics based off this, just as we'd do for test metrics. See [Batch Metrics](./batch_metrics.md) for more on this.

> NOTE:
> The ReSim API is generic and supports other cloud workloads, so the raw API endpoint for fetching test data is called the `jobs` endpoint. We will use the term test in this documentation unless referencing the API.


## Report Mode Computation

> WARNING:
> The information here represents the current status of the report metrics worker which is not final. We will improve the system by making it more generic and automated in the future

If the file `/tmp/resim/inputs/report_config.json` is present, we are in `report mode`! This file should look something like this:

```
{
  "authToken" : "...",
  "apiURL" : "https://api.resim.ai/v1",
  "reportID" : "7579affb-3e5b-4f02-871b-bf275aef67ee",
  "projectID": "a7a6f4fd-7852-4992-b073-4c89773ef953"
}
```

These four things will be used to retrieve the metrics data associated from batches with the API. This is made easier by various tooling we provide in this repository. See [Report Metrics](./report_metrics.md) for more on this.
