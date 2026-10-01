#!/bin/bash

# Usage: ./scripts/run.sh [options]
# Default: python3 -m orion_shadow.server

set -e

# Ensure we are in the project root
cd "$(dirname "$0")/../src"

# Default values (matching Orion SDK OrionComm.h)
HOST="0.0.0.0"
PORT="8745"
UDP_IN_PORT="8746"
TCP_PORT="8747"
DT="0.1"
DTED_PATH=""
TILE_URL=""
TILE_URL="https://mt0.google.com/vt/lyrs=m&x={x}&y={y}&z={z}" # Road map
TILE_URL="https://mt0.google.com/vt/lyrs=s&x={x}&y={y}&z={z}" # Satellite
TILE_URL="https://mt0.google.com/vt/lyrs=y&x={x}&y={y}&z={z}" # Satellite + Labels
FPS=""
TILE_ZOOM=""
MAX_TILE_ZOOM=""
PREFETCH_DISTANCE=""
NO_PREFETCH=""
LAT=""
LON=""
ALT=""
PAN=""
TILT=""
HEADING=""

LOGGER="INFO"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --host) HOST="$2"; shift ;;
        --port|--udp-port) PORT="$2"; shift ;;
        --udp-in-port) UDP_IN_PORT="$2"; shift ;;
        --tcp-port) TCP_PORT="$2"; shift ;;
        --dt) DT="$2"; shift ;;
        --logger|--log-level) LOGGER="$2"; shift ;;
        --dted-path) DTED_PATH="$2"; shift ;;
        --tile-url) TILE_URL="$2"; shift ;;
        --fps|--video-fps) FPS="$2"; shift ;;
        --tile-zoom|--zoom) TILE_ZOOM="$2"; shift ;;
        --max-tile-zoom) MAX_TILE_ZOOM="$2"; shift ;;
        --prefetch-distance) PREFETCH_DISTANCE="$2"; shift ;;
        --no-prefetch) NO_PREFETCH="1" ;;
        --lat|--latitude) LAT="$2"; shift ;;
        --lon|--longitude) LON="$2"; shift ;;
        --alt|--altitude) ALT="$2"; shift ;;
        --pan) PAN="$2"; shift ;;
        --tilt) TILT="$2"; shift ;;
        --heading) HEADING="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Construct command
CMD="python3 -m orion_shadow.server --host $HOST --port $PORT --udp-in-port $UDP_IN_PORT --tcp-port $TCP_PORT --dt $DT"

if [ -n "$LOGGER" ]; then
    CMD="$CMD --logger \"$LOGGER\""
fi

if [ -n "$DTED_PATH" ]; then
    CMD="$CMD --dted-path \"$DTED_PATH\""
fi

if [ -n "$TILE_URL" ]; then
    CMD="$CMD --tile-url \"$TILE_URL\""
fi

if [ -n "$FPS" ]; then
    CMD="$CMD --fps \"$FPS\""
fi

if [ -n "$TILE_ZOOM" ]; then
    CMD="$CMD --tile-zoom \"$TILE_ZOOM\""
fi

if [ -n "$MAX_TILE_ZOOM" ]; then
    CMD="$CMD --max-tile-zoom \"$MAX_TILE_ZOOM\""
fi

if [ -n "$PREFETCH_DISTANCE" ]; then
    CMD="$CMD --prefetch-distance \"$PREFETCH_DISTANCE\""
fi

if [ -n "$NO_PREFETCH" ]; then
    CMD="$CMD --no-prefetch"
fi

echo "Starting OrionShadow Simulator..."
echo "Command: $CMD"
echo "--------------------------------"

eval "$CMD"
