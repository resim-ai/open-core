# Test Suite Metrics

Test suite report metrics are quite distinct from test and batch-level metrics in the sense that they
are designed to offer a longitudinal perspective on a test suite.

> Example Report Metric 1: a line chart of a scalar batch metric, over each time it has been run

> Example Report Metric 2: a list of the experiences that have failed most frequently

## Report metrics mode

As mentioned in [the Metrics Builds docs](./metrics_builds.md), report metrics use a similar (potentially identical) Docker image as job and 
batch metrics. 

This image can differentiate whether it's computing in `job mode`, `batch mode`, or `report mode` , based off whether the file `/tmp/resim/inputs/report_config.json` is present. If it is present, it should be computing report metrics.

## Computing report metrics

The report metrics config (as provided to the report metrics run on launch) is a simple json with three fields: an auth token, an API URL, and a report ID.

```
{
  "authToken" : "...",
  "apiURL" : "https://api.resim.ai/v1",
  "reportID" : "7579affb-3e5b-4f02-871b-bf275aef67ee"
}
```

These fields should be used to retrieve the report and therefore the list of batches to be used to compute the report.


# Scrap
, and use these to compute batch-level metrics. We provide code to do this in [open-core](https://github.com/resim-ai/open-core), in combination with some code snippets below.

First you can read the config in using the following snippet:

```
import json 

with open(BATCH_METRICS_CONFIG_PATH, "r", encoding="utf-8") as metrics_config_file:
    metrics_config = json.load(metrics_config_file)

token=metrics_config["authToken"],
api_url=metrics_config["apiURL"],
batch_id=metrics_config["batchID"],
```

Once these are loaded, you can download the metrics using our `fetch_job_metrics` Python package.

```
import resim.metrics.fetch_job_metrics as fjm

job_to_metrics: Dict[uuid.UUID, UnpackedMetrics] = fjm.fetch_job_metrics_by_batch(token, api_url, uuid.UUID(batchID))
```

The result maps job IDs to `UnpackedMetrics` - this is a simple `dataclass` with three fields:

- `metrics: List[Metric]` - a list of all the metrics in that job
- `metrics_data: List[MetricsData]` - a list of all the metrics data in that job
- `names: Set[str]` - a set of all the names of Metrics and MetricsData present

In other words, it very simply gives you all the metrics and metrics data associated with each job. You then use these metrics and data in order to compute and write your batch metrics - (just as you did for job metrics!) - by following the instructions in the [Metrics writer docs](./metrics_writer.md). You write the output to the exact same place as before: `/tmp/resim/outputs/metrics.binproto`.

> NOTE: A common pattern is to write MetricsData in the job metrics *without an associated Metric*, and then use this data when it comes to computing batch metrics. This allows you to make new batch metrics without having associated job metrics.
