#!/bin/bash

# Master script to execute multiple test scripts
# Usage: ./master_test_runner.sh

# Define the array of script names (without .sh extension)
scripts=(
  # "busy_wait_priority"
  # "busy_wait"
  "scalability_tests"
  "scalability_priority_tests"
  # Add more script names as needed
)

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$SCRIPT_DIR"

echo "=== Master Test Runner ==="
echo "Scripts directory: $SCRIPTS_DIR"
echo "Timestamp: $(date)"
echo "=========================="

# Check if scripts directory exists
if [ ! -d "$SCRIPTS_DIR" ]; then
  echo "Error: Scripts directory not found at $SCRIPTS_DIR"
  exit 1
fi

# Loop through each script name
for script_name in "${scripts[@]}"; do
  script_path="$SCRIPTS_DIR/${script_name}.sh"
  
  echo ""
  echo "=== Executing: $script_name ==="
  echo "Script path: $script_path"
  
  # Check if script exists
  if [ ! -f "$script_path" ]; then
    echo "Warning: Script not found: $script_path"
    echo "Skipping to next script..."
    continue
  fi
  
  # Check if script is executable
  if [ ! -x "$script_path" ]; then
    echo "Making script executable: $script_path"
    chmod +x "$script_path"
  fi
  
  # Execute the script
  echo "Starting execution at: $(date)"
  start_time=$(date +%s)
  
  if "$script_path"; then
    end_time=$(date +%s)
    duration=$((end_time - start_time))
    echo "✓ Completed: $script_name (Duration: ${duration}s)"
  else
    echo "✗ Failed: $script_name"
  fi
  
  echo "Finished at: $(date)"
  echo "=========================="
done

echo ""
echo "=== All scripts completed ==="
echo "Final timestamp: $(date)"