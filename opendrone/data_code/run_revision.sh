#!/usr/bin/env bash
set -e
data_code_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_dir="$(cd -- "$data_code_dir/../.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup not found: $ros_setup (CSV verification can run directly with Python)." >&2
  exit 1
fi
source "$ros_setup"
export PYTHONPATH="$data_code_dir:$repository_dir/opendrone/scripts${PYTHONPATH:+:$PYTHONPATH}"
revision_workspace="$(python3 -c 'from revision_support.paths import workspace_root; print(workspace_root())')"
if [[ ! -f "$revision_workspace/devel/setup.bash" ]]; then
  echo "Set OPENDRONE_WORKSPACE to a built catkin workspace containing devel/setup.bash." >&2
  exit 1
fi
source "$revision_workspace/devel/setup.bash"
# Keep this checkout ahead of any differently named/older checkout in the overlay.
export PYTHONPATH="$data_code_dir:$repository_dir/opendrone/scripts${PYTHONPATH:+:$PYTHONPATH}"
export ROS_PACKAGE_PATH="$repository_dir${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
revision_artifacts="$(python3 -c 'from revision_support.paths import artifacts_root; print(artifacts_root())')"
export MPLCONFIGDIR="$revision_artifacts/.matplotlib"
export ROS_HOME="${ROS_HOME:-$revision_artifacts/.ros}"
cd "$revision_workspace"
exec python3 "$data_code_dir/${OPENDRONE_DATA_ENTRY:-run_revision.py}" "$@"
