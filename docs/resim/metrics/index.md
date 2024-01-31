# Overview

One of ReSim's core products is a comprehensive and growing metrics system. The purpose of this system is to allow users to display information from their simulation runs in an online dashboard, in a way such that they can be compared between different runs on different branches, and tracked over time. Metrics exist at two levels:

1. Job Metrics: Metrics are computed per-test-job, based off outputs from the simulation (such as log data and artifacts) - an example would be a *precision-recall* curve.
2. Batch Metrics: Multiple Job Metrics are aggregated to result in Batch Metrics. For example, an average accuracy across all the jobs in your batch. 

These metrics are written using the same system, which is the [ResimMetricsWriter](https://github.com/resim-ai/open-core/blob/main/resim/metrics/python/metrics_writer.py), although the "input data" is of course different between the two. 

To allow you to use this, we recommend working through the docs in this order:

# Contents

1. [Metrics Builds](./metrics_builds.md): how to make a metrics build to run your metrics.
1. [Metrics Data](./metrics_data.md): how to extract and represent data from your logs, ready to visualize in a metric.
1. [Metric Types](./metric_types.md): a summary of the types of metrics supported in the dashboard, and how to use them.
1. [Metrics Writer](./metrics_writer.md): how to write metrics in a way such that they appear in the dashboard
1. [Batch Metrics](./batch_metrics.md): how to compute batch metrics.
