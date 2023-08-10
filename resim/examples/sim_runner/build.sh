#!/bin/bash

set -e

SIM_RUNNER_DIR=resim/examples/sim_runner

# Build the binary for the simulator
bazel build //resim/simulator:resim_run
cp ./bazel-bin/resim/simulator/resim_run "${SIM_RUNNER_DIR}"

docker build resim/examples/sim_runner/ -t sim_runner:latest
