#pragma once

#include <memory>
#include <atomic>

#include <mrs_uav_status/tui/tui.hpp>

#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/uav_status_short.hpp>

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

private:
  void initialize();

  bool _profiler_enabled_ = false;

  enum class StatusState
  {
    STANDARD,
    REMOTE,
    GIMBAL,
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

  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>      sh_uav_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatusShort> sh_uav_status_short_;

  std::shared_ptr<TimerType> timer_status_fast_;
  std::shared_ptr<TimerType> timer_status_slow_;
  std::shared_ptr<TimerType> timer_resize_;

  void timerStatusFast();
  void timerStatusSlow();
  void timerResize();
  void callbackUavStatus(const mrs_msgs::msg::UavStatus::ConstSharedPtr msg);
  void callbackUavStatusShort(const mrs_msgs::msg::UavStatusShort::ConstSharedPtr msg);

  std::atomic<bool> initialized_ = false;

  mrs_lib::Profiler         profiler_;
  std::unique_ptr<tui::TUI> tui_;
};

} // namespace mrs_uav_status
