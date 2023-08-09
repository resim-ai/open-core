# Creating a Sim Runner Container

# Introduction
The contents of this folder are intended to demonstrate how to create a docker
image that can be run by ReRun such that it is afforded the correct experience
inputs by the worker container and such that the logs are correctly captured
and presented in the app. To get started, we'll be using the `resim_run` binary
as our representative simulator to be run in ReRun. Since we want to test this
runner image out locally, we will use a test script `test_run.sh` to stand in
for ReRun.

# Prerequisites

To run this example, it is assumed that you have cloned the resim-ai/open-core
repository and that you are operating inside of the development docker
container shipped with it. See the [getting started
guide](https://docs.resim.ai/) for more information.

# Running the example

The steps for running this example are as follows:

 - Clone and navigate into to the `open-core` repository.
```
git clone https://github.com/resim-ai/open-core.git
cd open-core
```
 - Start the development docker container.
```
./.devcontainer/run.sh
```
 - Build the sim runner image. This builds the `//resim/simulator:resim_run`
binary using bazel and then copies the resulting binary and `run_sim.sh` into
the `sim_runner:latest` docker image as it builds.
```
./resim/examples/sim_runner/build.sh
```
 - Run the sim runner image. This will set up input and output directories in
   `/root/` which are mounted to `/tmp/resim/` in the docker image when it is
   run. It also populates the `/root/inputs` directory with `experience.sim`,
   which is the scenario description for this experience. It then runs the
   `sim_runner:latest` container and prints the contents of the outputs
   directory.
```
./resim/examples/sim_runner/test_run.sh
```
