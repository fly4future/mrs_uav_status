#pragma once
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <memory>

// --- Internal Package Includes ---
#include <mrs_uav_status/ros/service.hpp>
#include <mrs_uav_status/ros/topic_info.hpp>

#include <mrs_uav_status/tui/system_info.hpp>
#include <mrs_uav_status/tui/control_bar.hpp>
#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/tui/status_window.hpp>
#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/tui/colors.hpp>
#include <mrs_uav_status/utils/split.hpp>
#include <mrs_uav_status/utils/terminal.hpp>

#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/uav_status_short.hpp>
#include <mrs_msgs/msg/gimbal_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>

namespace mrs_uav_status::tui
{
class TUI {
public:
  TUI(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_sc, const std::string &colorscheme, bool colorblind_mode, bool minimized_mode,
      const std::string &display_config_filename, const std::string &turbo_remote_constraints);

  // | --------------------- Data push (thread-safe) --------------------- |
  void onUavStatus(const mrs_msgs::msg::UavStatus &msg);
  void onUavStatusShort(const mrs_msgs::msg::UavStatusShort &msg);

  // | --------------------- Window Handlers -------------------- |
  void setupWindows(bool have_data);
  void stringHandler(WINDOW *win);
  void uavStateHandler(WINDOW *win);
  void nodeStatsHandler(WINDOW *win);
  void hwApiStateHandler(WINDOW *win);
  void generalInfoHandler(WINDOW *win);
  void genericTopicHandler(WINDOW *win);
  void controlManagerHandler(WINDOW *win);

private:
  std::string _colorscheme_;
  std::string _display_config_filename_;
  std::string _turbo_remote_constraints_;
  bool        _colorblind_mode_;
  bool        _minimized_mode_;
  bool        _light_ = false;

  // | ------------------------- ROS Core ----------------------- |
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  mrs_lib::PublisherHandler<mrs_msgs::msg::GimbalState>            ph_gimbal_state_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sc_goto_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_constraints_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_gains_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_estimator_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sc_hover_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ----------------------- UAV status snapshot --------------- |
  std::mutex               mutex_status_msg_;
  mrs_msgs::msg::UavStatus uav_status_;

  struct MenuRow
  {
    std::string           label;
    std::function<void()> on_open;
  };

  std::vector<MenuRow> main_menu_rows_;
  std::vector<MenuRow> sub_menu_rows_;

  // | -------------------- Remote & Flight --------------------- |
  void remoteHandler(int key, WINDOW *win);
  void gimbalHandler(int key, WINDOW *win);
  void remoteModeFly(const mrs_msgs::msg::Reference &ref_in);

  // | ------------------- Menu & Input Logic ------------------- |
  void        setupMainMenu();
  void        setupGotoMenu();
  void        setupDisplayMenu();
  void        setupDisplayText();
  bool        mainMenuHandler(int key_in);
  bool        gotoMenuHandler(int key_in);
  bool        displayMenuHandler(int key_in);
  static bool isValidMenuIndex(int index, size_t container_size);
  void        createSubMenu(std::vector<std::string> &submenu_entries);
  void        createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<mrs_msgs::srv::String> &service_client);
  void        createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> &service_client);

  // | ---------------------- TMUX & Misc ----------------------- |
  std::vector<int> selected_tmux_window_;
  std::string      session_name_;
  const int        MAX_SELECTED_TMUX_WINDOWS = 2;
  int              terminal_cols_ = 0, terminal_lines_ = 0;


  bool updateTermSize();
  void prefillUavStatus();
  void topLineHandler(WINDOW *win);


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

  // | ----------------------- Data Storage --------------------- |
  std::vector<tui::StatusWindow> menu_vec_;
  std::vector<tui::StatusWindow> submenu_vec_;
  std::vector<tui::ControlBar>   goto_menu_inputs_;

  std::vector<TopicInfo>   string_topic_;
  std::vector<Service>     service_vec_;
  std::vector<std::string> service_input_vec_;
  std::vector<std::string> main_menu_text_;
  std::vector<std::string> display_menu_text_;
  std::vector<std::string> goto_menu_text_;
  std::vector<double>      goto_double_vec_;
};
} // namespace mrs_uav_status::tui
