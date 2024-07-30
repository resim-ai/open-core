# Test Suite Report Metrics

Test suite report metrics are quite distinct from test and batch-level metrics in the sense that they
are designed to offer a longitudinal perspective on a test suite.

> Example Report Metric 1: a line chart of a scalar batch metric, over each time it has been run

> Example Report Metric 2: a list of the experiences that have failed most frequently

## Report metrics mode

As mentioned in [the Metrics Builds docs](./metrics_builds.md), report metrics use a similar (potentially 
identical) Docker image as job and batch metrics. 

This image can differentiate whether it's computing in `test mode`, `batch mode`, or `report mode` , based off 
whether the file `/tmp/resim/inputs/report_config.json` is present. If it is present, it should be computing
report metrics.

## Computing report metrics

The report metrics config (as provided to the report metrics run on launch) is a simple json with four fields: an auth token, an API URL, a project ID, and a report ID.

```
{
  "authToken" : "...",
  "apiURL" : "https://api.resim.ai/v1",
  "projectID" : "7579affb-3e5b-4f02-871b-bf275aef67ee",
  "reportID" : "9328c806-4f2a-41ca-abbc-7e28e1741384"
}
```

These fields should be used to retrieve the report and therefore the list of batches to be used to compute the report.
We provide code to do this in [open-core](https://github.com/resim-ai/open-core/tree/main/resim/metrics/default_report_metrics.py), in combination with some code snippets below.

First you can read the config in using the following snippet:

```
import json 

with open(REPORT_METRICS_CONFIG_PATH, "r", encoding="utf-8") as metrics_config_file:
    metrics_config = json.load(metrics_config_file)

token=metrics_config["authToken"],
api_url=metrics_config["apiURL"],
project_id=metrics_config["projectID"],
report_id=metrics_config["reportID"],
```

Once these are loaded, you can download the batches associated with the report 
using our `fetch_report_metrics` Python package.

```
import asyncio

from resim_python_client.client import AuthenticatedClient
import resim.metrics.fetch_report_metrics as frm

async def main():
    client = AuthenticatedClient(base_url=api_url, token=token)
    batches = await frm.fetch_batches_for_report(client, report_id, project_id)

if __name__ == '__main__':
    asyncio.run(main())
```
