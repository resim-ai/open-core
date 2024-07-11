# Overview

One of ReSim's core products is a comprehensive and growing **metrics framework** 
for evaluating the performance of Embodied AI applications, such as robots.
The purpose of this product is to allow users to display information from their simulation
runs of their embodied AI system in an online dashboard, in a way such that judgments can 
be rendered about its performance that can be compared between different versions of the
system on different branches, and tracked over time. 

Metrics exist at three levels:

1. Test Metrics: Metrics are computed per-test (i.e. per simulation), based off outputs from
the simulation (such as log data and artifacts) - an example would be a *precision-recall* 
curve across a set of input data.
1. Batch Metrics: Multiple test metrics are aggregated to result in batch metrics. For example, 
an *average accuracy* across all the tests in your batch. 
1. Test Suite Report Metrics: These metrics help you assess how the performance of your system 
has evolved over time. For example *average accuracy over time*. 

These metrics are written using the same system, which is the 
[ResimMetricsWriter](https://github.com/resim-ai/open-core/blob/main/resim/metrics/python/metrics_writer.py), 
although the "input data" is of course different between the three. 

The basic mechanism in the ReSim app is a `metrics build`, which is a docker image that
wraps any metrics code you write. The image is run by the ReSim app after every simulation. 

It is ReSim's core philosophy to allow maximum flexibility for engineers to display
the analysis that is most relevant to them. We aim to achieve this in three ways:

1. Open Source: The entirety of our metrics framework is open source to ensure transparency
for those using ReSim to test and evaluate their embodied AI system.
1. Determine your own aggregations: Rather than the ReSim web app providing fixed, limited 
aggregations of the results of a set of tests (batch metrics), or fixed longitudinal reports 
(test suite report metrics), the ReSim metrics framework lets you write code to decide how 
to aggregate. ReSim also provides some sensible default metrics to get users started.
1. Standard plotting libraries: While ReSim supports a range of custom metrics 
styles, described in [Metrics Types](./metric_types.md), any Plotly chart can also be wrapped
with our metrics metadata and displayed using [Plotly.JS](https://github.com/plotly/plotly.js).

To allow you to use this powerful framework, we recommend working through the docs in 
this order:

# Contents

1. [Metrics Builds](./metrics_builds.md): How to make a metrics build to run your metrics
and what data contract it expects.
1. [Metrics Data](./metrics_data.md): How to extract and represent data from your logs, 
ready to visualize in a metric.
1. [Metric Types](./metric_types.md): A summary of the types of metrics supported in 
the dashboard, and how to use them.
1. [Metrics Writer](./metrics_writer.md): How to write metrics in a way such that they 
appear in the dashboard.
1. [Events](./events.md): A summary of events, the assignment of metrics to events, 
and how to use them.
1. [Batch Metrics](./batch_metrics.md): How to compute batch metrics.
1. [Test Suite Reports](./report_metrics.md): How to compute test suite reports and their
metrics.
