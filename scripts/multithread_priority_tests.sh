#!/bin/bash

# Define your array of (publisher, subscriber) pairs
pairs=(
  "4 4"
  "10 5"
  "20 40"
  "100 200"
  "200 200"
  "400 100"
  # Add more pairs as needed
)

LOG_DIR="/home/guy/test_logs"
RESULT_FILE="$LOG_DIR/executor_timing_results.txt"

# Create timestamp directory
TIMESTAMP=$(date +"%Y_%m_%d_%H_%M")
LOG_DIR_WITH_TIME="$LOG_DIR/multithread_priority_async/$TIMESTAMP"
mkdir -p "$LOG_DIR_WITH_TIME"

# Number of threads to use for the multithread executor
THREADS=5

for pair in "${pairs[@]}"; do
  # Parse publisher and subscriber from the pair
  set -- $pair
  PUB=$1
  SUB=$2

  # 1. Delete the current results file
  rm -f "$RESULT_FILE"

  # 2. Run the multithread scalability test node
  ros2 run multithread_priority_test_node multithread_priority_test --publishers "$PUB" --subscribers "$SUB" --threads "$THREADS" --interval 1 --duration 60

  # 3. Copy the new results to a uniquely named file
  cp "$RESULT_FILE" "$LOG_DIR_WITH_TIME/multithread_scale_${PUB}_${SUB}_t${THREADS}.txt"

done