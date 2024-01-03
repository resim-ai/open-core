# The Metrics Writer

When authoring metrics, we provide a writer that lets you simply output all your metrics into our protobuf format, without dealing with protobuf or details of the file format. We call this the `ResimMetricsWriter`.

## Overall usage

An example usage is as follows:

```python
from resim.metrics.proto.validate_metrics_proto import validate_job_metrics
from resim.metrics.python.metrics_writer import ResimMetricsWriter

metrics_writer = ResimMetricsWriter(JOB_ID) # Make metrics writer!

# TODO: Add your metrics or metrics data here!
metrics_writer.add_metrics_data(...)
metrics_writer.add_metric(...)
# END TODO!

metrics_output = metrics_writer.write() # This gives you the protobuf to output!
validate_job_metrics(metrics_output.metrics_msg) # This should validate, if you wrote valid metrics!

# Finally, write that message to file as metrics.binproto.
with open("/tmp/resim/outputs/metrics.binproto", "wb") as f:
      f.write(metrics_output.metrics_msg.SerializeToString())
```

For transparency, we provide the output as a `protobuf` message for you to write to file (rather than directly writing it to file) as in our experience it can be useful to inspect the output protobuf, and (for example) write it to text protobuf so you can inspect it manually.

## How to write metrics

The easiest way to write a new metric to the `ResimMetricsWriter` is to use the fluent API provided by the `Metric` class.

Below is an example, making a `DoubleOverTimeMetric` using the fluent API - for more info on what this is, see [the Metrics Types docs](./metric_types.md).

```python

# Example metrics data - not necessarily written to metrics writer
TIMESTAMPS: MetricsData = ...
ERROR_INDEXED_BY_TIMESTAMPS: MetricsData = ...
STATUSES_INDEXED_BY_TIMESTAMPS: MetricsData = ...

# Example write
metrics_writer
      .add_double_over_time_metric("Localization error") # Type and name specified here
      .with_description("Accumulated error in localization over time")
      .append_doubles_over_time_data(ERROR_INDEXED_BY_TIMESTAMPS) # Append a single data series, as we only want to plot one
      .append_statuses_over_time_data(STATUSES_INDEXED_BY_TIMESTAMPS) # Append associated timestamps
      .with_failure_definitions([DoubleFailureDefinition(fails_below=0.0, fails_above=1.0)])
      .with_start_time(Timestamp(secs=0))
      .with_end_time(Timestamp(secs=10))
      .with_y_axis_name("Error (%)")
      .with_legend_series_names(["Error"])
      .with_status(MetricStatus.PASSED_METRIC_STATUS)
      .with_importance(MetricImportance.HIGH_IMPORTANCE)
      .with_should_display(True)
      .with_blocking(False)
```

This will write a double over time metric to our output, with all the desired data and properties. As a rule of thumb, most of the parameters described in [the Metrics Types docs](./metric_types.md) can be set using the fluent API by appending `with_` to it.

> NB: Notice that the `MetricsData` we used does *not* necessarily have to be written to the metrics writer. Provided the top-level metric referencing that data is written to the metrics writer, any associated data will also be written. For example, even the `TIMESTAMPS` data will (transitively) be written by the above code.

### Writing metrics data
You can also manually write `MetricsData` to the metrics_writer using 

```python
metrics_writer.add_metrics_data(TIMESTAMPS)
```

This is useful, because we may want to write data that is not immediately referenced by any metric. Such data may be used by **batch metrics**. The `MetricsData` class also has a simple fluent API to make this easier.

## Validating the metrics writer output

It's important to validate your output, to check it's a valid protobuf message that our system can plot. We currently provide this through the `validate_job_metrics` function in `resim.metrics.proto.validate_metrics_proto`. Note that this is called on the **output metrics message** of the metrics writer (i.e. `output.metrics_msg`), not the metrics writer itself.
