#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mrs_uav_status
{

struct TopicStatus
{
  rclcpp::Time            last_time;
  int                     counter     = 0;
  double                  window_rate = 0.0;
  std::vector<double>     rates;
  size_t                  rates_iterator = 0;
  rclcpp::Node::SharedPtr node           = nullptr;

  // `node_in` is optional; when null, `last_time` is initialized to zero ROS time.
  TopicStatus(double window_rate_in, int buffer_len, rclcpp::Node::SharedPtr node_in = nullptr)
      : last_time(node_in ? node_in->get_clock()->now() : rclcpp::Time(0, 0, RCL_ROS_TIME)), counter(0), window_rate(window_rate_in),
        rates(buffer_len * static_cast<int>(window_rate_in), 0.0), rates_iterator(0), node(node_in) {
  }
};

} // namespace mrs_uav_status
