# ReSim's Open Libraries

[![codecov](https://codecov.io/gh/resim-ai/open-core/branch/main/graph/badge.svg?token=URJ09ULAH4)](https://codecov.io/gh/resim-ai/open-core)

This repository contains the open source subset of ReSim's C++ code intended to
accelerate robotics development. For detailed documentation, please visit
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
