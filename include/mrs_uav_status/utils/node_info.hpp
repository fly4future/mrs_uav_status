#pragma once

#include <string>

namespace mrs_uav_status
{

struct NodeInfo
{
  std::string node_name;
  int         node_pid       = 0;
  float       node_cpu_usage = 0.0f;
  long        last_utime     = 0;
  long        last_stime     = 0;

  explicit NodeInfo(const std::string &node_name_in) : node_name(node_name_in) {
  }
};

} // namespace mrs_uav_status
