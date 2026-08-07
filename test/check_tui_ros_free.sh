#!/usr/bin/env bash
# Fails if tui/ (or the utils/ headers it depends on) pull in ROS message/service/Node types,
# directly or transitively. rclcpp::Time/Clock/Duration (local UI timing only, no
# node/topic/service graph access) is an explicitly allowed exception -- see
# .superpowers/specs/2026-08-06-ros-status-tui-layering-design.md.
#
# Deliberately does NOT use `set -e`: grep's exit status (0 = match found, 1 = no match,
# >1 = error) is inspected explicitly below so "no forbidden includes" and "grep itself failed"
# (e.g. a scanned directory went missing in a rename) are never conflated.
set -uo pipefail

ROOT="$1"

# Matches both angle-bracket and quoted #include forms for each forbidden ROS package
# (`#include <mrs_msgs/...>` or `#include "mrs_msgs/..."`), plus rclcpp/node.hpp and the
# rclcpp.hpp umbrella header that pulls it in transitively, plus the ROS-handle helper headers
# that imply a live node/topic/service graph.
FORBIDDEN='#include[[:space:]]*[<"](mrs_msgs|std_msgs|std_srvs|rclcpp/node|rclcpp/rclcpp)|service_client_handler\.h|subscriber_handler\.h|publisher_handler\.h'

# utils/ is in scope too: tui/ includes utils/ headers directly, so a forbidden dependency
# hiding in utils/ is just as much a layering violation as one written directly under tui/.
DIRS=(
  "$ROOT/include/mrs_uav_status/tui"
  "$ROOT/src/tui"
  "$ROOT/include/mrs_uav_status/utils"
)
if [ -d "$ROOT/src/utils" ]; then
  DIRS+=("$ROOT/src/utils")
fi

for d in "${DIRS[@]}"; do
  if [ ! -d "$d" ]; then
    echo "ERROR: expected directory '$d' does not exist -- check_tui_ros_free.sh's scanned paths are stale (rename?)." >&2
    exit 1
  fi
done

matches=$(grep -rnE "$FORBIDDEN" "${DIRS[@]}" 2>&1)
grep_status=$?

if [ "$grep_status" -eq 0 ]; then
  echo "$matches" >&2
  echo "ERROR: tui/ (and the utils/ headers it depends on) must not depend on ROS message/service/Node types." >&2
  exit 1
elif [ "$grep_status" -gt 1 ]; then
  echo "$matches" >&2
  echo "ERROR: grep failed while scanning for forbidden includes (exit $grep_status)." >&2
  exit 1
fi

echo "OK: tui/ (and the utils/ headers it depends on) have no ROS message/service/Node dependency."
exit 0
