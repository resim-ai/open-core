# Metrics Data

Metrics data is series-like data extracted from logs, in a way that can be referenced by metrics, and represented in the app. 

Within the metrics SDK, the basic object is a `SeriesMetricsData`, which is fundamentally a  numpy `array` of arbitrary length, with a unique `name` associated to it. This numpy `array` can contain arbitrary types, but as of right now, only four types of data can actually be written to the metrics output:

1. `float`
2. `string`
3. `resim.python.metrics_utils.Timestamp`
4. `resim.python.metrics_utils.MetricStatus`

Attempts to write any other types will fail. 

> (To store `int` types, it is currently recommended to convert to `float`. To store `enum`, it is recommended to convert to `string`.)

Here is an example of code making a `SeriesMetricsData`

```
from resim.metrics.python.metrics import SeriesMetricsData
from resim.metrics.python.metrics_utils import Timestamp

# Example timestamp data - in practice this would be extracted from log
timestamps = np.array(
  [Timestamp(secs=i, nanos=0) for i in range(100)]
) 

# Make SeriesMetricsData from array
timestamps_data = SeriesMetricsData(
  name="Log timestamps",
  series = timestamps,
  unit = "Timestamp" # optional
)
```

We also support a "fluent API" style for declaring `MetricsData`, which becomes more useful when trying to make new metrics and data and write them at the same time. The above would be equivalent to:

```
timestamps_data = (
  SeriesMetricsData(name = "Log timestamps")
    .with_series(timestamps)
    .with_unit("Timestamp")
)
```

# Indexing data

In order to provide a "table-like" functionality, we support the idea of one array `index`-ing another. This means that one array is considered the index of another.

As an example, we could add the following code to the above:

```
floats = np.array([random.random() for i in range(100)])

floats_data = SeriesMetricsData(
  name = "floats",
  series = floats,
  unit = "/ 1.0",
  index_data = timestamps_data,
)
```

This is now an array "indexed" by the timestamps we made earlier, so can be thought of as a "dictionary" of the form:

```
{
  timestamps[0]: floats[0],
  timestamps[1]: floats[1],
  ...
  timestamps[100]: floats[100]
}
```

By indexing multiple series with the same single index, tables can easily be converted into our format.

> Note: There are a few key details here:
> 1. The index array and data array must have the **same length**.
> 2. Index arrays should **not** themselves be indexed.
> 3. If the index array contains repeats, then the first matching index is considered the relevant one when trying to retrieve variables by index.

# Grouping metrics data

We support the notion of `grouped` metrics data, which is effectively the output of a `GROUP BY` operation. That is, it is one array "grouped" by the values in the another array of the same length. These arrays should generally share the same index, if one is present.

For example:

```
state_set = ['ACCELERATING', 'BRAKING', 'STOPPED']

states = np.array(
  [random.choice(state_set) for i in range(100)]
)

states_data = SeriesMetricsData(
  name = "states",
  series = states,
  index = timestamps
)

grouped_floats_data = floats_data.group_by(states_data)

```

The output of this can be thought of as dictionary of the form:

```
grouped_floats_data = {
  "ACCELERATING": [floats],
  "BRAKING": [floats],
  "STOPPED": [floats]
}
```

> NB: When *writing* grouped data to ResimMetricsWriter, we currently only support grouping by `string`. 

> NB: GroupedMetricsData can be indexed, and should be indexed by other GroupedMetricsData, with the same groups and lengths!

> Tip: Generally, if you put GroupedMetricsData into a chart, it will make a "list" of charts - one for each group present. You can pick which chart you want using a dropdown menu. Some of these are not yet fully implemented in the UI yet. See the [Metrics types docs](./metric_types.md) for more on this.

## Using metrics data

Metrics data are *referenced* by metrics, in order to create charts that are then shown in the UI. You can think of this as passing an array into `plt.plot(...)` or similar. To see the types of metrics, and what data they use, see [the Metric Types docs](./metric_types.md). To see how to write these metrics, see [the Metrics Writer docs](./metrics_writer.md).
