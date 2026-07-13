#pragma once

#include <memory>
#include <atomic>

#include <mrs_uav_status/tui/tui.hpp>

#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <std_msgs/msg/string.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/param_loader.h>

#if USE_ROS_TIMER == 1
using TimerType = mrs_lib::ROSTimer;
#else
using TimerType = mrs_lib::ThreadTimer;
#endif

namespace mrs_uav_status
{

class Status : public mrs_lib::Node {

public:
  Status();
  ~Status();

private:
  void initialize();

  bool _profiler_enabled_ = false;

  enum class StatusState
  {
    STANDARD,
    REMOTE,
    MAIN_MENU,
    GOTO_MENU,
    DISPLAY_MENU
  };

  StatusState state_ = StatusState::STANDARD;

  rclcpp::Node::SharedPtr          node_;
  rclcpp::Clock::SharedPtr         clock_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>       sh_general_robot_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>    sh_state_estimation_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>            sh_control_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo> sh_collision_avoidance_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>                sh_uav_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>       sh_system_health_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::State>                  sh_uav_state_;
  mrs_lib::SubscriberHandler<std_msgs::msg::String>                 sh_display_string_;

  std::shared_ptr<TimerType> timer_render_;

  rclcpp::Duration slow_period_{0, 0};
  rclcpp::Duration resize_period_{0, 0};
  rclcpp::Time     last_slow_run_;
  rclcpp::Time     last_resize_check_;

  void timerRender();
  void callbackGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr msg);
  void callbackStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr msg);
  void callbackControlInfo(const mrs_msgs::msg::ControlInfo::ConstSharedPtr msg);
  void callbackCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr msg);
  void callbackUavInfo(const mrs_msgs::msg::UavInfo::ConstSharedPtr msg);
  void callbackSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr msg);
  void callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg);
  void callbackDisplayString(const std_msgs::msg::String::ConstSharedPtr msg);

  std::atomic<bool> initialized_ = false;

  mrs_lib::Profiler         profiler_;
  std::unique_ptr<tui::TUI> tui_;
};

} // namespace mrs_uav_status
