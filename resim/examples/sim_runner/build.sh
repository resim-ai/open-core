#!/bin/bash

set -e

SIM_RUNNER_DIR=resim/examples/sim_runner

# Build the binary for the simulator
bazel build //resim/simulator:resim_run
cp ./bazel-bin/resim/simulator/resim_run "${SIM_RUNNER_DIR}"

# Build the image. The name sim_runner is arbitrary here and it does not have to
# correspond to the ECR we'll eventually push to when creating a build. We will
# re-tag it before pushing.
docker build resim/examples/sim_runner/ -t sim_runner:latest
