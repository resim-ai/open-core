# ROS2

The [Robot Operating System](https://www.ros.org/) or ROS and its associated
libraries represent one of the most ubiquitous collections of robotics software
in use today. We therefore provide first class support for using ROS2 with
ReSim's tools. To enable this without making ReSim's tools explicitly depend on
ROS2 libraries, we provide a converter binary that can be used to convert ROS2
bags to the [ReSim Log Format](../msg) which our other tools and libraries
natively consume. 

## Basic Conversion

If you're using common ROS2 message types (e.g. those in
`std_msgs` or `geometry_msgs`) to log your topics of interest the conversion is
as simple as cloning [resim/open-core](https://github.com/resim-ai/open-core/)
and running the following commands.

```bash
bazel run //resim/ros2:convert_log -- \
    --log </path/to/my/rosbag_dir/>   \
    --output </path/to/my/converted/log.mcap>
```

If there are common public types that you need which we don't yet support, feel
free to reach out and we can add them.

A pybound version of this converter is
also provided in the `resim_ros2` package distributed through our open-core
[Releases](https://github.com/resim-ai/open-core/releases) and can be used like
so:

```python
import resim.ros2.resim_log_from_ros2_python as rlr2
from resim.ros2 import RESIM_DIR

# ...

rlr2.resim_log_from_ros2(
    str(RESIM_DIR / "ros2" / "default_converter_plugin.so"), # See plugin info below
    input_log_path,
    converted_log_path)
```

## Custom Message Types & Converter Plugins

Occasionally, it is necessary or just convenient to convert a custom message
type to a serialized ReSim protobuf type so that its contents can be used with
ReSim's open source metrics and analysis tools. To facilitate this, the
aforementioned converter tools are designed with a plugin architecture to allow
custom converters to be written. The core of the plugin interface is the
following function:

```c
extern "C" ReSimConverterPluginStatus resim_convert_ros2_to_resim(
    const char *const ros2_message_type,
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message);
```

This essentially converts serialized ros2 message bytes to serialized resim
message bytes based on the message type given as a string. There are also other
functions that the binary needs to interrogate the plugin for:

 * Which types are supported for conversion.
 * Descriptions of the serialization format for the converted type (as required
   by Foxglove/MCAP tools).
   
The best practice here is to simply copy
[resim/ros2/default_converter_plugin.cc](https://github.com/resim-ai/open-core/blob/main/resim/ros2/default_converter_plugin.cc)
and add the messages you care about to it, following its example. One can also
create a minimal plugin which dynamically loads the default converter plugin and
uses it to handle all the common message types. Once you've compiled such a
plugin, it can be provided using the `--plugin` flag for `convert_log`:

```bash
bazel run //resim/ros2:convert_log --          \
    --plugin </path/to/my/custom/converter.so> \
    --log </path/to/my/rosbag_dir/>            \
    --output </path/to/my/converted/log.mcap>
```

or in Python:
```python
import resim.ros2.resim_log_from_ros2_python as rlr2
from resim.ros2 import RESIM_DIR

# ...

rlr2.resim_log_from_ros2(
    my_custom_converter_plugin_path,
    input_log_path,
    converted_log_path,
```
