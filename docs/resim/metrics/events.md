# Introduction

The ReSim metrics SDK and web app supports the creation of so-called **events** to aid the
analysis of a robot's performance in a simulation.

An event is a time-centered occurrence of some importance to a robotics developer, which can 
occur at a timestamp and have metrics specifically associated with it. In comparison to the
usual ReSim metrics types e.g. line chart of velocity over time, or scalar precision/recall
values, an event is centered around a timestamp and allows the user to highlight parts of the
simulation of most interest and metrics relevant to them.

As a simple example, consider an autonomous mower, processing a large field. A metrics build
could generate an event every time the mower got within a safe distance to a pedestrian
*and* show snapshots from the system state as metrics.

In this way, we can generate a timeline of important parts of a simulation.

# Registering Events

In the metrics SDK, events can be created in the same way as normal metrics. Assuming
you already have a `MetricsWriter`:

```python

# Example event write
event = metrics_writer
        .add_event("My Unique Event Name") # Event specified here
        .with_description("An optional description of an event")
        .with_tags(["collision", "pedestrian"]) # A list of tags that can be used to categorize the event 
        .with_timestamp(Timestamp(secs=10)) # Specify the timestamp in your simulation that the event occurs
        .with_status(MetricStatus.PASSED_METRIC_STATUS)
        .with_importance(MetricImportance.HIGH_IMPORTANCE)
```

In addition, a `with_metrics(...)` function expects you to pass a list of metrics objects that 
you have previously created to associate with the event. The metrics will not be displayed outside 
the context of the event and need to be registered as event metrics. For example:

```python
# Create two event metrics
metric_1 = metrics_writer
            .add_double_over_time_metric("Localization error")
            # ...
            .is_event_metric()

metric_2 = metrics_writer
            .add_scalar_metric("Total performance")
            # ...
            .is_event_metric()

# Now associate with the event.
event.with_metrics(metric_1, metric_2)
```
