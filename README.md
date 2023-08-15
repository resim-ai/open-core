# ReSim's Open Libraries

[![codecov](https://codecov.io/gh/resim-ai/open-core/branch/main/graph/badge.svg?token=URJ09ULAH4)](https://codecov.io/gh/resim-ai/open-core)

This repository contains the open source subset of
[ReSim](https://www.resim.ai/)'s C++ code intended to accelerate robotics
development. For detailed documentation, please visit
[docs.resim.ai](https://docs.resim.ai).

# Getting Started 
The expected developer workflow takes place entirely within our Docker
container. See [docs.resim.ai](https://docs.resim.ai) for detailed instructions
on getting started.

# Structure
Currently, the libraries are divided up into:

 - [Examples](resim/examples): Example binaries which demonstrate how to use our libraries.
 - [Assert](resim/assert): A simple library that we use for runtime assertions.
 - [Utils](resim/utils): Common utilities used throughout the repository.
 - [Math](resim/math): Common math libraries.
 - [Time](resim/time): Common time libraries.
 - [Transforms](resim/transforms): Libraries for 3D transforms.
 - [Curves](resim/curves): Libraries for curves through 3D space.
 - [Actor](resim/actor/state): Libraries for rigid body states and trajectories.
 - [Visualization](resim/visualization): Libraries for visualizing 3D objects
   with [ReSim View](https://docs.resim.ai/visualization/).
 - [Testing](resim/testing): Common libraries for testing.
 - [Auth](resim/auth): Device code client for use with oauth.
 - [Dynamics](resim/dynamics): Libraries for defining and integrating dynamics.
 - [Experiences](resim/experiences): The definition of our experience
   description format.
 - [Geometry](resim/geometry): Libraries for geometric primitives and
   algorithms.
 - [Metrics](resim/metrics): Tools for computing metrics during and after
   simulations.
 - [Planning](resim/planning): Tools for high-level planning and optimal
   control.
 - [Simulator](resim/simulator): Our core simulator libraries. 


# Building

We use [Bazel](https://bazel.build/) as our build and test tool. As an example
for those who might be less familiar with bazel, one can build our simulator
using the command:

```
bazel build //resim/simulator:resim_run
```

This will result in the binary for the simulator being placed at
`open-core/bazel-bin/resim/simulator/resim_run`. The binary can either be run
directly or run using `bazel run` instead of `bazel build` above.

Our build is *mostly* hermetic, and consequently most of our dependencies are
tracked through bazel and will be automatically downloaded and built when needed
for the target being built. However, we do depend on the UUID library from
[util-linux/util-linux](https://github.com/util-linux/util-linux), which is
included in our development docker container.
