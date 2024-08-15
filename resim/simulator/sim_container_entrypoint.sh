#!/bin/sh

set -e

# Run the simulation with the experience and return result.mcap:
echo "Running the simulation..."
./resim/simulator/resim_run -c /tmp/resim/inputs/experience.sim -l /tmp/resim/outputs/resim_log.mcap "$@"

# Add visualization content
echo "Adding Visualizations..."
./resim/visualization/log/make_visualization_log -l /tmp/resim/outputs/resim_log.mcap -o /tmp/resim/outputs/vis.mcap
