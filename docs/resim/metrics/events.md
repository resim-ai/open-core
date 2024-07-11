# Introduction

The ReSim metrics SDK and web app supports the creation of so-called **events** to aid the
analysis of an Embodied AI system's performance in a simulation.

An event is a time-centered occurrence of some importance to an engineer, which can 
occur at a timestamp and have metrics specifically associated with it. In comparison to the
usual ReSim metrics types e.g. line chart of velocity over time, or scalar precision/recall
values, an event is centered around a timestamp and allows the engineer to highlight parts of 
the simulation of most interest and metrics relevant to them.

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
        .with_relative_timestamp(Timestamp(secs=10)) # Specify the timestamp in your simulation that the event occurs
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

The ReSim Metrics Validator checks that all metrics associated with an event have been flagged as event metrics
via the `is_event_metric()` function.

## Available Parameters

Aside from the list of metrics, the available parameters for an event are described below:

- `name: str` - A required, immutable name. The same name cannot be used twice, so they are uniquely identifying.
- `description: str` - A string description of the event.
- `timestamp: Timestamp` - A seconds and nanos representation of the timestamp that the event occurs at. In the SDK, this
can be interpreted as either a `relative` (to the start of the simulation) or an `absolute` timestamp via `with_absolute_timestamp()`
and `with_relative_timestamp()`. The most common use case is for relative times e.g. 3s into the simulation, but for processing
real-world logs, the absolute timestamp is more relevant.
- `tags: list[str]` - An optional list of tags associated with the event, that can be used to filter and organize on the web app.
- `status: MetricStatus` - An overall status (e.g. PASSED, FAIL_BLOCK) for the event itself. Note that this is not, by default, computed from
the statuses of any associated metrics: it needs to be computed by the user
- `importance: MetricImportance` - An overall importance (e.g. CRITICAL, HIGH, LOW, ZERO).
