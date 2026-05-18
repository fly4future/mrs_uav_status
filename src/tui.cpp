#include <mrs_uav_status/tui/tui.hpp>

namespace mrs_uav_status::tui
{

TUI::TUI(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_sc, const std::string &colorscheme, bool colorblind_mode, bool minimized_mode,
         const std::string &display_config_filename, const std::string &turbo_remote_constraints)
    : _colorscheme_(colorscheme),
      _display_config_filename_(display_config_filename),
      _turbo_remote_constraints_(turbo_remote_constraints),
      _colorblind_mode_(colorblind_mode),
      _minimized_mode_(minimized_mode),
      node_(node),
      clock_(node->get_clock()) {

  _light_ = (colorscheme.find("COLORSCHEME_LIGHT") != std::string::npos);

  ph_gimbal_state_    = mrs_lib::PublisherHandler<mrs_msgs::msg::GimbalState>(node_, "~/gimbal_command_out");
  sc_goto_reference_  = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/reference_out", cbkgrp_sc);
  sc_set_constraints_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_constraints_out", cbkgrp_sc);
  sc_set_gains_       = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_gains_out", cbkgrp_sc);
  sc_set_controller_  = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_controller_out", cbkgrp_sc);
  sc_set_tracker_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_tracker_out", cbkgrp_sc);
  sc_set_estimator_   = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_estimator_out", cbkgrp_sc);
  sc_hover_           = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/hover_out", cbkgrp_sc);

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);
}

void TUI::onUavStatus(const mrs_msgs::msg::UavStatus &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  uav_status_ = msg;
}

void TUI::onUavStatusShort(const mrs_msgs::msg::UavStatusShort &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  uav_status_.odom_x     = msg.odom_x;
  uav_status_.odom_y     = msg.odom_y;
  uav_status_.odom_z     = msg.odom_z;
  uav_status_.odom_hdg   = msg.odom_hdg;
  uav_status_.odom_color = msg.odom_color;
  uav_status_.odom_hz    = msg.odom_hz;
  uav_status_.cmd_x      = msg.cmd_x;
  uav_status_.cmd_y      = msg.cmd_y;
  uav_status_.cmd_z      = msg.cmd_z;
  uav_status_.cmd_hdg    = msg.cmd_hdg;
}

bool TUI::updateTermSize() {

  bool changed = false;

  std::string command  = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string response = utils::callTerminal(command.c_str());

  std::vector<std::string> results;

  results = mrs_uav_status::utils::splitByChar(response, 'x');

  int cols, lines;

  try {
    cols  = std::stoi(results[0]);
    lines = std::stoi(results[1]);
  }

  catch (const std::invalid_argument &e) {
    cols  = 0;
    lines = 0;
  }

  if (terminal_cols_ != cols || terminal_lines_ != lines) {
    terminal_lines_ = lines;
    terminal_cols_  = cols;
    changed         = true;
  }

  return (changed);
}

void TUI::setupWindows(bool have_data) {

  std::string command = "tmux display-message -p '#S'";
  session_name_       = utils::callTerminal(command.c_str());
  session_name_.erase(std::remove(session_name_.begin(), session_name_.end(), '\n'), session_name_.end());

  command                           = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string              response = utils::callTerminal(command.c_str());
  std::vector<std::string> results;
  results = mrs_uav_status::utils::splitByChar(response, 'x');

  if (_minimized_mode_) {

    control_manager_window_ = newwin(4, 9, 1, 1);
    uav_state_window_       = newwin(6, 9, 5, 1);
    top_bar_window_         = newwin(1, 140, 0, 1);
    general_info_window_    = newwin(4, 9, 1, 10);
    hw_api_state_window_    = newwin(6, 9, 5, 10);
    debug_window_           = newwin(terminal_lines_ - 15, terminal_cols_ - 1, 13, 1);
    generic_topic_window_   = newwin(10, 9, 1, 19);
    string_window_          = newwin(10, 15, 1, 28);
    bottom_window_          = newwin(1, 120, 11, 1);

  } else {

    uav_state_window_       = newwin(7, 26, 5, 1);
    control_manager_window_ = newwin(4, 26, 1, 1);
    hw_api_state_window_    = newwin(7, 25, 5, 27);
    general_info_window_    = newwin(4, 25, 1, 27);
    top_bar_window_         = newwin(1, 140, 0, 1);
    bottom_window_          = newwin(1, 120, 12, 1);
    debug_window_           = newwin(terminal_lines_ - 15, terminal_cols_ - 1, 13, 1);
    int half_lines          = (terminal_lines_ - 18) / 2;
    sub_tmux_window_1_      = derwin(debug_window_, half_lines, terminal_cols_ - 3, 1, 1);
    sub_tmux_window_2_      = derwin(debug_window_, half_lines, terminal_cols_ - 3, half_lines + 2, 1);

    generic_topic_window_ = newwin(11, 25, 1, 52);
    string_window_        = newwin(11, 32, 1, 77);
    node_stats_window_    = newwin(11, 50, 1, 109);
  }

  clear();
  _light_ = tui::setupColors(have_data, _colorscheme_, _colorblind_mode_);
}

void TUI::generalInfoHandler(WINDOW *win, bool mini) {
  werase(win);
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  wattroff(win, A_STANDOUT);

  bool   avoiding_collision, can_takeoff, null_tracker;
  double cpu_load, cpu_ghz, free_ram, total_ram;
  int    free_hdd;
  {
    std::scoped_lock lock(mutex_status_msg_);
    avoiding_collision = uav_status_.avoiding_collision;
    can_takeoff        = uav_status_.automatic_start_can_takeoff;
    null_tracker       = uav_status_.null_tracker;
    cpu_load           = uav_status_.cpu_load;
    cpu_ghz            = uav_status_.cpu_ghz;
    free_ram           = uav_status_.free_ram;
    total_ram          = uav_status_.total_ram;
    free_hdd           = uav_status_.free_hdd;
  }

  printBox(win, avoiding_collision, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  printCpuLoad(win, cpu_load, mini);
  printMemLoad(win, free_ram, total_ram, mini);
  if (!mini) {
    printCpuFreq(win, cpu_ghz);
  }
  printDiskSpace(win, free_hdd, last_gigas_, mini);
  last_gigas_ = free_hdd;

  wnoutrefresh(win);
}


} // namespace mrs_uav_status::tui
