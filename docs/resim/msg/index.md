# ReSim Log Format

<!--TODO(michael) Switch out metrics link when metrics docs exist.-->

The ReSim app is designed to work with any log format. This way, you can ensure
that your simulation jobs produce the same types of logs that you might have
from real-world data collection. This is often essential for tool or library
re-use. In ReSim's open source library, however, we are working to provide a
rich set of tools for computing and visualizing salient
[metrics](https://github.com/resim-ai/open-core/tree/main/resim/metrics). We
further aim to continue producing tools to analyze and leverage data from both
simulation and real-world logs in the open source library. As we cannot write
such tools to work with any log format, we maintain our own **ReSim Log
Format**. We want this format to be as simple and portable as possible so that
converters from other common robotics log formats are as trivial as possible. To
this end we define the format like so:

The ReSim Log Format is based on Foxglove's
[MCAP](https://foxglove.dev/blog/introducing-the-mcap-file-format) file
format. Using this file format furnishes us with a pre-existing set of open
source tools and libraries for reading, writing, and visualizing our logs. In
particular, [Foxglove Studio](https://foxglove.dev/) natively visualizes this
format efficiently.

Those familiar with the MCAP format will be aware that this format can hold
arbitrary serialized robotics data with ROS1, ROS2, JSON, Flatbuffers, and
Protobuf recieving first-class support from Foxglove's open-source
libraries. Therefore, we define a set of protobuf message schemas in
[open-core/resim/msg](https://github.com/resim-ai/open-core/tree/main/resim/msg)
which our tools are designed to recognize and work with efficiently. This allows
all of ReSim's metrics libraries to be leveraged for the small cost of writing
log converters. For the ubiquitous [ROS2](https://ros.org/) middleware log
format (CDR), such converters are [provided](../ros2/) for the defined ReSim
message types. With that said, we have deliberately chosen to promulgate our own
log format rather than using ROS's so that our users are not forced to depend on
this large software ecosystem if they would prefer not to. Simultaneously, our
provided converters eliminate integration friction for ROS users.

A ReSim log therefore consists of an MCAP file containing ReSim messages for
topics of interest. Other topics may be present and may have any
schema/serialization. These topics will generally be ignored by ReSim tools, or
copied unmodified by tools that input and output mcap files. There may be
exceptions to this in the future.
