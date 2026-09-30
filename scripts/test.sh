#!/bin/bash

# Usage: ./scripts/test.sh [test_file_or_dir]
# Example: ./scripts/test.sh tests/test_camera.py

set -e

# Ensure we are in the project root
cd "$(dirname "$0")/.."

# Check if argument is provided
if [ -z "$1" ]; then
    echo "No test target provided. Running all tests with pytest..."
    pytest
else
    echo "Running tests for: $1"
    pytest "$1"
fi
