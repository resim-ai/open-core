# Batch Metrics

Batch metrics are very similar to test metrics, but are computed after all the test metrics for a given batch are computed, using the test-level metrics and metrics data as the "input."

> Example Batch Metric 1: a weighted average of a test-level scalar metric, across all the tests present.

> Example Batch Metric 2: a histogram of a single series that is computed by all tests, after merging these series into one by appending.


## Batch metrics mode

As mentioned in [the Metrics Builds docs](./metrics_builds.md), batch metrics use the same single Docker image as test metrics. 

This image can differentiate whether it's computing in `test mode` or `batch mode`, based off whether the file `/tmp/resim/inputs/batch_metrics_config.json` is present. If it is present, it should be computing batch metrics.

## Computing batch metrics

The batch metrics config (as provided to the batch metrics run on launch) is a simple json with three fields: an auth token, an API URL, and a batch ID.

```
{
  "authToken" : "...",
  "apiURL" : "https://api.resim.ai/v1",
  "batchID" : "7579affb-3e5b-4f02-871b-bf275aef67ee"
}
```

These fields should be used to retrieve the test-level metrics and metrics data associated with a batch, and use these to compute batch-level metrics. We provide code to do this in [open-core](https://github.com/resim-ai/open-core), in combination with some code snippets below.

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

> Note: A test is called a `job` in the raw API, so the functions are named in this way for consistency with the API.

```
import resim.metrics.fetch_job_metrics as fjm

job_to_metrics: Dict[uuid.UUID, UnpackedMetrics] = fjm.fetch_job_metrics_by_batch(token, api_url, uuid.UUID(batchID))
```

The result maps job IDs to `UnpackedMetrics` - this is a simple `dataclass` with three fields:

- `metrics: List[Metric]` - a list of all the metrics in that test
- `metrics_data: List[MetricsData]` - a list of all the metrics data in that test
- `names: Set[str]` - a set of all the names of Metrics and MetricsData present

In other words, it very simply gives you all the metrics and metrics data associated with each test. You then use these metrics and data in order to compute and write your batch metrics - (just as you did for test metrics!) - by following the instructions in the [Metrics writer docs](./metrics_writer.md). You write the output to the exact same place as before: `/tmp/resim/outputs/metrics.binproto`.

> NOTE: A common pattern is to write MetricsData in the test metrics *without an associated Metric*, and then use this data when it comes to computing batch metrics. This allows you to make new batch metrics without having associated test metrics.
