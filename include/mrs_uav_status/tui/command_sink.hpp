#pragma once

#include <functional>
#include <string>
#include <vector>

namespace mrs_uav_status::tui
{

// Outbound ROS actions TUI can trigger, injected by ros_status so TUI never links against ROS
// client-library service/message types. Every call is synchronous.
struct CommandSink
{
  struct ServiceResult
  {
    bool        success = false;
    std::string message;
  };

  // One config-driven Trigger-backed menu entry (from mrs_uav_status/menu/service_list).
  struct NamedService
  {
    std::string                    display_name;
    std::function<ServiceResult()> call;
  };

  std::function<ServiceResult(double x, double y, double z, double heading, const std::string &frame_id)> sendGoto;
  // Fire-and-forget, matches today's behavior (the response is not checked at the call site).
  std::function<void(double vx, double vy, double vz, double heading_rate, const std::string &frame_id)> sendVelocityReference;
  std::function<ServiceResult(const std::string &value)>                                                 setConstraints;
  std::function<ServiceResult(const std::string &value)>                                                 setGains;
  std::function<ServiceResult(const std::string &value)>                                                 setController;
  std::function<ServiceResult(const std::string &value)>                                                 setTracker;
  std::function<ServiceResult(const std::string &value)>                                                 setEstimator;
  std::function<ServiceResult()>                                                                         hover;
  std::function<ServiceResult()>                                                                         toggleOutput;

  std::vector<NamedService> extra_services;

  // Wall-clock "now" in seconds, straight from the node clock. Not a service call, but routed
  // through CommandSink for the same reason as everything else here: it lets TUI read the node
  // clock without linking against any ROS client-library type itself. Used where the tick-start
  // snapshot_.now_seconds is too stale -- e.g. stamping renderServiceResult()'s clear-time after
  // a blocking service call returns.
  std::function<double()> nowSeconds;
};

} // namespace mrs_uav_status::tui
