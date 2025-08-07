#!/bin/bash

LOG_DIR="/home/guy/test_logs/"
RESULT_FILE="$LOG_DIR/executor_timing_results.txt"

# Create timestamp directory
TIMESTAMP=$(date +"%Y_%m_%d_%H_%M")
LOG_DIR_WITH_TIME="$LOG_DIR/pairtalk/$TIMESTAMP/"
mkdir -p "$LOG_DIR_WITH_TIME"

# Run 3 iterations
for iteration in {1..3}; do
  # 1. Delete the current results file
  rm "$RESULT_FILE"

  # 2. Run the node for 80 seconds
  ros2 run pairtalk pairtalk

  # 3. Copy the new results to a uniquely named file
  cp "$RESULT_FILE" "$LOG_DIR_WITH_TIME/pubsub_${iteration}.txt"

done