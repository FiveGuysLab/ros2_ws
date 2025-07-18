#!/bin/bash

# Define your array of (publisher, subscriber) pairs
pairs=(
  "1 1"
  "2 2"
  "5 10"
  # Add more pairs as needed
)

LOG_DIR="/home/guy/test_logs/scalability_test_node"
RESULT_FILE="$LOG_DIR/executor_timing_results.txt"

# Create timestamp directory
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR_WITH_TIME="$LOG_DIR/$TIMESTAMP"
mkdir -p "$LOG_DIR_WITH_TIME"

for pair in "${pairs[@]}"; do
  # Parse publisher and subscriber from the pair
  set -- $pair
  PUB=$1
  SUB=$2

  # 1. Delete the current results file
  rm -f "$RESULT_FILE"

  # 2. Run the scalability test node
  ros2 run scalability_test_node scalability_test --publishers "$PUB" --subscribers "$SUB" --interval 1 --duration 60

  # 3. Copy the new results to a uniquely named file
  cp "$RESULT_FILE" "$LOG_DIR_WITH_TIME/scale_${PUB}_${SUB}.txt"

  # 4. Delete the results file for the next iteration
  rm -f "$RESULT_FILE"
done