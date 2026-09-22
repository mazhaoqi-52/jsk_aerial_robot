#!/usr/bin/env bash
# Usage: play_experiment.sh 1|2|3|/path/to/file.bag [start:=0] [rate:=1] [pause:=true]
set -eo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../../.." && pwd)
source /opt/ros/one/setup.bash
export ROS_PACKAGE_PATH="$repo_dir:${ROS_PACKAGE_PATH:-}"
# Dedicated local master: the playback topics stay separate from flight sessions.
export ROS_MASTER_URI=http://localhost:11321
export ROS_HOSTNAME=localhost
unset ROS_IP
selection=${1:-1}
if (( $# )); then shift; fi
case "$selection" in
  1) bag=/home/ma/rosbag/2026-04-17-18-27-35_push_wall_20N_success.bag ;;
  2) bag=/home/ma/rosbag/2026-07-30-20-10-15_push_fail_push_and_slide_success.bag ;;
  3) bag=/home/ma/rosbag/2026-07-30-20-29-19_push_and_slide_10N_success.bag ;;
  *) bag=$selection ;;
esac
if [[ ! -f "$bag" ]]; then echo "Bag not found: $bag" >&2; exit 1; fi
echo "Opening $bag. Press SPACE in this terminal to pause/resume; Ctrl+C to close."
exec roslaunch -p 11321 beetle_omni rosbag_visualization.launch "bag:=$bag" "$@"
