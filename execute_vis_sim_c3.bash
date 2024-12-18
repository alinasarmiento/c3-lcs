#!/bin/bash

# Run visualizer
./bazel-bin/examples/pushbot/pushbot_visualizer &

# Run controller
./bazel-bin/examples/pushbot/pushbot_c3_controller &

# Run simulator
./bazel-bin/examples/pushbot/pushbot_sim &

# Wait for all background jobs to finish
wait
