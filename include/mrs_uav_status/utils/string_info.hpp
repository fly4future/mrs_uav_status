#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace mrs_uav_status
{

struct StringInfo
{
  std::string  publisher_name;
  std::string  id;
  std::string  display_string;
  bool         persistent = false;
  rclcpp::Time last_time;

  StringInfo(rclcpp::Time last_time_in, const std::string &publisher_name_in, const std::string &display_string_in, const std::string &id_in,
             bool persistent_in)
      : publisher_name(publisher_name_in), id(id_in), display_string(display_string_in), persistent(persistent_in), last_time(last_time_in) {
  }
};

} // namespace mrs_uav_status
