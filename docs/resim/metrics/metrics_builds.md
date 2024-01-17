# Metrics Builds

## What are Metrics Builds

Metrics builds are code which is run *after* your simulation runs, taking experience and log data, and writing it to a format that the ReSim app is able to plot in app dashboard. Unlike simulations, we mostly expect these to be run in **Python** at present, and therefore currently only provide Python tooling.

## How to register Metrics Builds

Just like regular simulation builds, you need to make a docker image and register metrics builds with the ReSim CLI before you can run them. When registered, they should appear in the ReSim app, in a separate dropdown shown when launching an experience. For more on registering metrics builds, see [Run batches with metrics](https://docs.resim.ai/run-metrics-batches/) in the ReRun docs. For the purpose of this doc, we'll assume you have such a docker image registered, and just talk about what it should do.

## Job Mode vs Batch Mode

Metrics Builds have two modes, both of which are implemented within the same single metrics build.

1. `job mode` - in this mode, we compute metrics associated with a single job, based off output data from the job.
2. `batch mode` - in this mode, we compute metrics associated with a batch of jobs, based off output metrics data from `job mode`. 

Whether we are in `job mode` or `batch mode` is determined by the files in `/tmp/resim/inputs/`. In particular, we are in `batch mode` if and only if the file `/tmp/resim/inputs/batch_metrics_config.json` is present. (If not, then the folder will contain the experience and log data associated with a single job.) 

# Output

Regardless of the mode, the job should finish by writing to `/tmp/resim/outputs/metrics.binproto`, in a format complying with the [JobMetrics object](https://github.com/resim-ai/open-core/blob/main/resim/metrics/proto/metrics.proto), and which validates with the [Python validator](https://github.com/resim-ai/open-core/blob/main/resim/metrics/proto/validate_metrics_proto.py) we provide. This is in fact the only requirement for a metrics build. 

> WARNING:
> It is highly recommended you do not interact with this proto directly, and instead use the metrics SDK documented in [Metrics Data](./metrics_data.md), [Metric Types](./metrics_data.md), and [Metric Types](./metric_types.md).
 
## Job Mode computation

Assuming the file `/tmp/resim/inputs/batch_metrics_config.json` is not present, we are in `job mode`! We then have two relevant folders:

1. `/tmp/resim/inputs/logs/` - this contains any log data output by the simulation job.
2. `/tmp/resim/inputs/experience/` - this contains all the experience data associated with the simulation job.

We would read these in, and as always, we'd output any metrics we want to `/tmp/resim/outputs/metrics.binproto`. For more on how to actually write these metrics see [Metrics Data](./metrics_data.md), [Metric Types](./metrics_data.md), and [Metric Types](./metric_types.md).

## Batch Mode computation

> WARNING:
> The information here represents the current status of the batch metrics worker which is not final. We hope to make this system somewhat more generic and less manual in the near future.

If the file `/tmp/resim/inputs/batch_metrics_config.json` is present, we are in `batch mode`! This file should look something like this:

```
{
  "authToken" : "...",
  "apiURL" : "https://api.resim.ai/v1",
  "batchID" : "7579affb-3e5b-4f02-871b-bf275aef67ee"
}
```

These three things will be used to retrieve the metrics data associated from jobs with the API. This is made easier by various tooling we provide in this repo. We then compute metrics based off this, just as we'd do for job metrics. See [Batch Metrics](./batch_metrics.md) for more on this.
