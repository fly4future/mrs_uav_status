#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace mrs_uav_status
{

class TopicInfo {
public:
  TopicInfo();
  
  TopicInfo(rclcpp::Node::SharedPtr node, double window_rate, int buffer_length, double desired_rate);
  
  TopicInfo(rclcpp::Node::SharedPtr node, double window_rate, int buffer_length, double desired_rate, 
            const std::string& topic_name, const std::string& topic_display_name);

  std::string                 getTopicName() const;
  std::string                 getTopicDisplayName() const;
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

} // namespace mrs_uav_status
