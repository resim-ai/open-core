#!/bin/bash
#!/bin/bash

set -e

SIM_RUNNER_DIR=resim/examples/sim_runner

# Set up the experience in local folders in a volume which the dev container #
# mounts at /root/.

INPUTS=/root/inputs
OUTPUTS=/root/outputs

mkdir -p "${INPUTS}" "${OUTPUTS}"
cp "${SIM_RUNNER_DIR}/experience.sim" "${INPUTS}"

# The ReRun worker will run a command similar to this, ensuring that
# /tmp/resim/inputs and /tmp/resim/outputs are available in the runner.
docker run \
    -v root-home:/tmp/resim \
    sim_runner:latest

echo "Outputs:"
ls "${OUTPUTS}"
