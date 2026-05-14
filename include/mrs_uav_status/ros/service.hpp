#pragma once

#include <string>

#include <std_srvs/srv/trigger.hpp>

#include <mrs_lib/service_client_handler.h>

namespace mrs_uav_status
{

struct Service
{
  std::string service_name;
  std::string service_display_name;

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> service_client;

  Service(const std::string &name_in, const std::string &display_name_in) : service_name(name_in), service_display_name(display_name_in) {
  }
};

} // namespace mrs_uav_status
