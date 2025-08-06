#!/bin/bash

# Get the directory of the script, regardless of where it's called from
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Move to the target directory
TARGET_DIR="$SCRIPT_DIR/../../src/prolog_reasoning/know_plan_reasoning"

# Download the target files
curl -o "$TARGET_DIR/plan_comparison.pl" https://raw.githubusercontent.com/albertoOA/know_plan/main/src/prolog_reasoning/comparative/plan_comparison.pl
curl -o "$TARGET_DIR/__init__.pl" https://raw.githubusercontent.com/albertoOA/know_plan/main/src/prolog_reasoning/comparative/__init__.pl

echo "File updated!"
