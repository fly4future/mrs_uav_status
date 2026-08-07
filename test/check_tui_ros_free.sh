#!/usr/bin/env bash
# Fails if tui/ depends on ROS message/service/Node types. rclcpp::Time/Clock (local UI timing
# only, no node/topic/service graph access) is an explicitly allowed exception -- see
# .superpowers/specs/2026-08-06-ros-status-tui-layering-design.md.
set -euo pipefail

ROOT="$1"
FORBIDDEN='#include <mrs_msgs|#include <std_msgs|#include <std_srvs|#include <rclcpp/node|service_client_handler\.h|subscriber_handler\.h|publisher_handler\.h'

if grep -rnE "$FORBIDDEN" "$ROOT/include/mrs_uav_status/tui" "$ROOT/src/tui"; then
  echo "ERROR: tui/ must not depend on ROS message/service/Node types." >&2
  exit 1
fi

echo "OK: tui/ has no ROS message/service/Node dependency."
exit 0
