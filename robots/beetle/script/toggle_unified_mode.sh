#!/bin/bash
# Toggle unified_control_mode for all beetle modules.
# Usage:
#   ./toggle_unified_mode.sh true          # enable
#   ./toggle_unified_mode.sh false         # disable
#   ./toggle_unified_mode.sh true 1,2,3   # specify module ids

MODE=${1:-true}
IDS=${2:-"1,2,3"}

IFS=',' read -ra ID_ARRAY <<< "$IDS"
for id in "${ID_ARRAY[@]}"; do
  rosparam set /beetle${id}/controller/unified_control_mode "$MODE"
  echo "[beetle${id}] unified_control_mode = $MODE"
done
