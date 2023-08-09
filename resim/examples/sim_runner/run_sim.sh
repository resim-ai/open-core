#!/bin/bash

set -e

INPUTS=/tmp/resim/inputs
OUTPUTS=/tmp/resim/outputs

./resim_run --config "${INPUTS}/experience.sim" --log "${OUTPUTS}/sim_log.mcap"
