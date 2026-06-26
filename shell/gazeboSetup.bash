#!/bin/sh

set -eu

usage() {
    echo "Usage: $0 <PX4_DIR>"
    echo "Example: $0 /home/test/PX4-Autopilot"
}

if [ "${1-}" = "-h" ] || [ "${1-}" = "--help" ]; then
    usage
    exit 0
fi

if [ "$#" -ne 1 ]; then
    usage
    exit 1
fi

PX4_DIR=$1
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
SITL_CONFIG_DIR="${SCRIPT_DIR}/../opendrone/sitl_config"

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4 directory not found: $PX4_DIR"
    exit 1
fi

if [ ! -d "$SITL_CONFIG_DIR" ]; then
    echo "Error: SITL config directory not found: $SITL_CONFIG_DIR"
    exit 1
fi

# Support both new and old PX4 gazebo-classic layouts.
if [ -d "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic" ]; then
    PX4_GAZEBO_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
elif [ -d "$PX4_DIR/Tools/sitl_gazebo" ]; then
    PX4_GAZEBO_DIR="$PX4_DIR/Tools/sitl_gazebo"
else
    echo "Error: PX4 gazebo SITL directory not found in: $PX4_DIR"
    echo "Expected one of:"
    echo "  $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    echo "  $PX4_DIR/Tools/sitl_gazebo"
    exit 1
fi

echo "PX4 directory         : $PX4_DIR"
echo "SITL config directory : $SITL_CONFIG_DIR"
echo "Gazebo target dir     : $PX4_GAZEBO_DIR"

cp -r "$SITL_CONFIG_DIR"/models/* "$PX4_GAZEBO_DIR/models/"
cp -f "$SITL_CONFIG_DIR"/worlds/* "$PX4_GAZEBO_DIR/worlds/"
cp -f "$SITL_CONFIG_DIR"/launch/* "$PX4_DIR/launch/"

BUILD_PLUGIN_DIR=""
if [ -d "$PX4_DIR/build/px4_sitl_default/build_gazebo-classic" ]; then
    BUILD_PLUGIN_DIR="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic"
elif [ -d "$PX4_DIR/build/px4_sitl_default/build_gazebo" ]; then
    BUILD_PLUGIN_DIR="$PX4_DIR/build/px4_sitl_default/build_gazebo"
fi

if [ -d "$SITL_CONFIG_DIR/plugins" ]; then
    if [ -n "$BUILD_PLUGIN_DIR" ]; then
        cp -f "$SITL_CONFIG_DIR/plugins"/*.so "$BUILD_PLUGIN_DIR/" 2>/dev/null || true
        echo "Copied plugins (.so) to: $BUILD_PLUGIN_DIR"
    else
        echo "Warning: PX4 build plugin directory not found, skipped copying .so plugins."
        echo "Build PX4 SITL first, then run this script again to deploy plugins."
    fi
fi

echo "SITL gazebo files copied successfully."
