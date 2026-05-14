#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrs_lib/service_client_handler.h>

namespace mrs_uav_status
{

class TopicInfo {
public:
  TopicInfo();
  TopicInfo(rclcpp::Node::SharedPtr node, double window_rate_in, int buffer_len, double desired_rate_in);
  TopicInfo(rclcpp::Node::SharedPtr node, double window_rate_in, int buffer_len, double desired_rate_in, std::string topic_name_in,
            std::string topic_display_name_in);

  std::string                 getTopicName();
  std::string                 getTopicDisplayName();
  std::tuple<double, int16_t> getHz();
  void                        count();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time            last_time_;
  int                     counter_;
  std::string             topic_name_;
  std::string             topic_display_name_;
  double                  window_rate_;
  double                  desired_rate_;
  std::vector<double>     rates_;
  size_t                  rates_iterator_;
};

struct Service
{
  std::string service_name;
  std::string service_display_name;

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> service_client;

  Service(std::string name_in, std::string display_name_in) {
    service_name         = name_in;
    service_display_name = display_name_in;
  }
};

struct TopicStatus
{
  rclcpp::Time            last_time;
  int                     counter;
  double                  window_rate;
  std::vector<double>     rates;
  size_t                  rates_iterator = 0;
  rclcpp::Node::SharedPtr node;

  // `node_in` is optional; when null, `last_time` is initialized to zero ROS time.
  TopicStatus(double window_rate_in, int buffer_len, rclcpp::Node::SharedPtr node_in = nullptr) {
    node        = node_in;
    window_rate = window_rate_in;
    rates.resize(buffer_len * int(window_rate));
    rates.assign(rates.size(), 0.0);
    rates_iterator = 0;
    if (node) {
      last_time = node->get_clock()->now();
    } else {
      last_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    counter = 0;
  }
};

struct StringInfo
{
  std::string  publisher_name;
  std::string  id;
  std::string  display_string;
  bool         persistent;
  rclcpp::Time last_time;

  StringInfo(rclcpp::Time last_time, std::string publisher_name_in, std::string display_string_in, std::string id_in, bool persistent_in) {
    publisher_name  = publisher_name_in;
    display_string  = display_string_in;
    id              = id_in;
    persistent      = persistent_in;
    this->last_time = last_time;
  }
};

struct NodeInfo
{
  std::string node_name;
  int         node_pid;
  float       node_cpu_usage;
  long        last_utime;
  long        last_stime;

  NodeInfo(std::string node_name_in) {
    node_name      = node_name_in;
    node_pid       = 0;
    node_cpu_usage = 0.0;
    last_utime     = 0;
    last_stime     = 0;
  }
};

} // namespace mrs_uav_status
