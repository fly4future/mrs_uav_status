#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>
#include <form.h>

/* #include <utility> */
#include <tuple>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <rclcpp/rclcpp.hpp>

/* #include <iostream> */
/* #include <fstream> */
/* #include <thread> */

#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/uav_status_short.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/msg/constraint_manager_diagnostics.hpp>

#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/msg/custom_topic.hpp>

#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/service_client_handler.h>

#include <tf2_msgs/msg/tf_message.hpp>

inline constexpr int kKeyEnt    = 10;
inline constexpr int kKeyEsc    = 27;
/* inline constexpr int kKeyBackspace = 263; */
inline constexpr int kKeyDelete = 330;

inline constexpr int kColorPairNormal    = 100;
inline constexpr int kColorPairField     = 101;
inline constexpr int kColorPairGreen     = 102;
inline constexpr int kColorPairRed       = 103;
inline constexpr int kColorPairYellow    = 104;
inline constexpr int kColorPairBlue      = 105;
inline constexpr int kColorPairAlwaysRed = 106;

inline constexpr int kBackgroundDefault   = -1;
inline constexpr int kBackgroundTrueBlack = 16;

inline constexpr int kColorNiceRed   = 196;
inline constexpr int kColorDarkRed   = 88;
inline constexpr int kColorNiceGreen = 82;
inline constexpr int kColorDarkGreen = 2;
inline constexpr int kColorNiceBlue  = 33;
inline constexpr int kColorDarkBlue  = 19;
inline constexpr int kColorNiceYellow = 220;
inline constexpr int kColorDarkYellow = 172;

inline constexpr int kBufferSecsLen = 4;

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

  TopicStatus(double window_rate_in, int buffer_len) {
    window_rate = window_rate_in;
    rates.resize(buffer_len * int(window_rate));
    rates.assign(rates.size(), 0.0);
    rates_iterator = 0;
    if (node) {
      last_time = node->get_clock()->now();
    } else {
      last_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    counter        = 0;
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
