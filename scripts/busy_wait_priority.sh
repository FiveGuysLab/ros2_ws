#!/bin/bash

LOG_DIR="/home/guy/test_logs/busywait_priority_node"
RESULT_FILE="$LOG_DIR/executor_timing_results.txt"

# Create timestamp directory
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR_WITH_TIME="$LOG_DIR/$TIMESTAMP"
mkdir -p "$LOG_DIR_WITH_TIME"

# Run 3 iterations
for iteration in {1..3}; do
  # 1. Delete the current results file
  rm -f "$RESULT_FILE"

  # 2. Run the node for 80 seconds
  timeout 80s ros2 run busywait_priority_node busywait_priority_node &
  NODE_PID=$!
  
  # Wait for 80 seconds
  sleep 80
  
  # Kill the node if it's still running
  kill $NODE_PID 2>/dev/null

  # 3. Copy the new results to a uniquely named file
  cp "$RESULT_FILE" "$LOG_DIR_WITH_TIME/bw_priority_${iteration}.txt"

  # 4. Delete the results file for the next iteration
  rm -f "$RESULT_FILE"
done