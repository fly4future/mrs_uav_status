#pragma once

// --- Standard Includes ---
#include <fstream>
#include <filesystem>
#include <functional>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>

// --- Internal Package Includes ---
#include <mrs_uav_status/ros/service.hpp>
#include <mrs_uav_status/ros/topic_info.hpp>
#include <mrs_uav_status/tui/colors.hpp>
#include <mrs_uav_status/tui/system_info.hpp>
#include <mrs_uav_status/tui/control_bar.hpp>
#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/tui/status_window.hpp>
#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/tui/tui.hpp>
#include <mrs_uav_status/utils/split.hpp>
#include <mrs_uav_status/utils/terminal.hpp>

// --- ROS Msg & Srv Includes ---
#include <mrs_msgs/msg/node_cpu_load.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/gimbal_state.hpp>
#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/uav_status_short.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <std_srvs/srv/trigger.hpp>

// --- MRS Library Includes ---
#include <mrs_lib/node.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
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

  // | ----------------------- Parameters ----------------------- |
  std::string _uav_type_;
  std::string _uav_name_;
  std::string _colorscheme_;
  std::string _pwd_;
  std::string _display_config_filename_;
  bool        _colorblind_mode_  = false;
  bool        _profiler_enabled_ = false;

  // | -------------------- State Management -------------------- |
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

  // | ------------------------- ROS Core ----------------------- |
  rclcpp::Node::SharedPtr          node_;
  rclcpp::Clock::SharedPtr         clock_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>      sh_uav_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatusShort> sh_uav_status_short_;

  // | --------------------- Timers & Callbacks ------------------ |
  std::shared_ptr<TimerType> timer_status_fast_;
  std::shared_ptr<TimerType> timer_status_slow_;
  std::shared_ptr<TimerType> timer_resize_;

  void timerStatusFast();
  void timerStatusSlow();
  void timerResize();
  void callbackUavStatus(const mrs_msgs::msg::UavStatus::ConstSharedPtr msg);
  void callbackUavStatusShort(const mrs_msgs::msg::UavStatusShort::ConstSharedPtr msg);


  // | --------------------- Window Handlers -------------------- |
  void setupWindows();

  // | ---------------------- Window Pointers ------------------- |
  WINDOW *uav_state_window_       = nullptr;
  WINDOW *control_manager_window_ = nullptr;
  WINDOW *hw_api_state_window_    = nullptr;
  WINDOW *top_bar_window_         = nullptr;
  WINDOW *bottom_window_          = nullptr;
  WINDOW *generic_topic_window_   = nullptr;
  WINDOW *node_stats_window_      = nullptr;
  WINDOW *general_info_window_    = nullptr;
  WINDOW *debug_window_           = nullptr;
  WINDOW *sub_tmux_window_1_      = nullptr;
  WINDOW *sub_tmux_window_2_      = nullptr;
  WINDOW *string_window_          = nullptr;

  std::mutex               mutex_status_msg_;
  mrs_msgs::msg::UavStatus uav_status_;

  // | -------------------- Stats & Counters -------------------- |
  std::atomic<bool> initialized_ = false;
  bool              mini_        = false;
  bool              help_active_ = false;
  int               cols_ = 0, lines_ = 0;

  // | ---------------------- Misc ------------------------------ |
  bool updateTermSize();
  void prefillUavStatus();

  mrs_lib::Profiler profiler_;

  std::unique_ptr<tui::TUI> tui_;
};

} // namespace mrs_uav_status
