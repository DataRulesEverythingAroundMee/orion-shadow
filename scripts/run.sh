#!/bin/bash

# Usage: ./scripts/run.sh [options]
# Default: python3 -m orion_shadow.server

set -e

# Ensure we are in the project root
cd "$(dirname "$0")/.."

# Default values
HOST="0.0.0.0"
PORT="5000"
DT="0.1"
DTED_PATH=""

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --host) HOST="$2"; shift ;;
        --port) PORT="$2"; shift ;;
        --dt) DT="$2"; shift ;;
        --dted-path) DTED_PATH="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Construct command
CMD="python3 -m orion_shadow.server --host $HOST --port $PORT --dt $DT"

if [ -n "$DTED_PATH" ]; then
    CMD="$CMD --dted-path $DTED_PATH"
fi

echo "Starting OrionShadow Simulator..."
echo "Command: $CMD"
echo "--------------------------------"

eval "$CMD"
