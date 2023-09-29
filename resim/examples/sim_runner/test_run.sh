#!/bin/bash

set -e

SIM_RUNNER_DIR=resim/examples/sim_runner

# Set up the experience inputs and outputs in docker volumes
docker container create \
    --name tmp \
    --volume test_inputs:/inputs \
    --volume test_outputs:/outputs \
    busybox > /dev/null

# Copy the experience config into the test_inputs volume
docker cp "${SIM_RUNNER_DIR}/experience.sim" tmp:/inputs > /dev/null

# This run command ensures that /tmp/resim/inputs and /tmp/resim/outputs are
# mounted in the runner, which fits ReRun's contract.
docker run \
    -v test_inputs:/tmp/resim/inputs \
    -v test_outputs:/tmp/resim/outputs \
    sim_runner:latest

# Copy the outputs back
docker cp tmp:/outputs . > /dev/null

echo "Outputs:"
ls outputs/

docker rm tmp > /dev/null
