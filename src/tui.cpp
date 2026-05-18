#include <mrs_uav_status/tui/tui.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>

#include <mrs_lib/geometry/cyclic.h>

namespace mrs_uav_status::tui
{

using radians = mrs_lib::geometry::radians;


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

  last_time_got_data_       = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_time_got_short_data_ = rclcpp::Time(0, 0, clock_->get_clock_type());

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
  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_ = msg;
  }
  last_time_got_data_ = clock_->now();
}

void TUI::onUavStatusShort(const mrs_msgs::msg::UavStatusShort &msg) {
  {
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
  last_time_got_short_data_ = clock_->now();
}

void TUI::tickSlowCounter() {
  increment_counter_ = !increment_counter_;
  estimator_display_counter_ += int(increment_counter_);
  if (estimator_display_counter_ >= 3) {
    estimator_display_counter_ = 0;
  }
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

void TUI::stringHandler(WINDOW *win, bool mini) {
  std::vector<std::string> string_vector;
  bool   avoiding_collision, can_takeoff, null_tracker;
  uint8_t gnss_fix_type, gnss_num_sats;
  double  gnss_pos_acc, gnss_status_rate;

  {
    std::scoped_lock lock(mutex_status_msg_);
    string_vector      = uav_status_.custom_string_outputs;
    avoiding_collision = uav_status_.avoiding_collision;
    can_takeoff        = uav_status_.automatic_start_can_takeoff;
    null_tracker       = uav_status_.null_tracker;
    gnss_fix_type      = uav_status_.hw_api_gnss_fix_type;
    gnss_num_sats      = uav_status_.hw_api_gnss_num_sats;
    gnss_pos_acc       = uav_status_.hw_api_gnss_pos_acc;
    gnss_status_rate   = uav_status_.hw_api_gnss_status_hz;
  }

  if (gnss_status_rate > 0.0) {
    std::string fix_string;

    if (gnss_fix_type < 1 || gnss_fix_type >= 8) {
      fix_string += "-r ";
    }
    fix_string += "Fix Type: ";

    switch (gnss_fix_type) {
    case 0: fix_string += "NO GPS"; break;
    case 1: fix_string += "NO FIX"; break;
    case 2: fix_string += "2D FIX"; break;
    case 3: fix_string += "3D FIX"; break;
    case 4: fix_string += "3D SBAS FIX"; break;
    case 5: fix_string += "RTK FLOAT"; break;
    case 6: fix_string += "RTK FIX (INT)"; break;
    case 7: fix_string += "STATIC - BASESTATION"; break;
    case 8: fix_string += "PPP 3D FIX"; break;
    default: fix_string += "UNKNOWN"; break;
    }

    std::string gnss_acc_string;
    if (gnss_pos_acc >= 100.0) {
      gnss_acc_string = "N/A";
    } else {
      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << gnss_pos_acc;
      gnss_acc_string = stream.str();
    }
    std::string acc_string = "Num sats: " + std::to_string(gnss_num_sats) + " Acc: " + gnss_acc_string + " m";

    string_vector.push_back(fix_string);
    string_vector.push_back(acc_string);
  }

  if (string_vector.empty()) {
    werase(win);
    wnoutrefresh(win);
    return;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  for (unsigned long i = 0; i < string_vector.size(); i++) {

    int         tmp_color          = static_cast<int>(ColorPair::Normal);
    bool        blink              = false;
    std::string tmp_display_string = string_vector[i];

    if (tmp_display_string.at(0) == '-') {

      if (tmp_display_string.at(1) == 'r') {
        tmp_color = static_cast<int>(ColorPair::Red);
      } else if (tmp_display_string.at(1) == 'R') {
        tmp_color = static_cast<int>(ColorPair::Red);
        blink     = true;
      } else if (tmp_display_string.at(1) == 'y') {
        tmp_color = static_cast<int>(ColorPair::Yellow);
      } else if (tmp_display_string.at(1) == 'Y') {
        tmp_color = static_cast<int>(ColorPair::Yellow);
        blink     = true;
      } else if (tmp_display_string.at(1) == 'g') {
        tmp_color = static_cast<int>(ColorPair::Green);
      } else if (tmp_display_string.at(1) == 'G') {
        tmp_color = static_cast<int>(ColorPair::Green);
        blink     = true;
      }

      if (tmp_color != static_cast<int>(ColorPair::Normal)) {
        tmp_display_string.erase(0, 3);
      }
    }

    if (blink) {
      wattron(win, A_BLINK);
    }

    wattron(win, COLOR_PAIR(tmp_color));

    if (mini) {
      printCompressedLimitedString(win, (i) + 1, 1, tmp_display_string, 15);
    } else {
      printLimitedString(win, (i) + 1, 1, tmp_display_string, 30);
    }

    wattroff(win, COLOR_PAIR(tmp_color));
    wattroff(win, A_BLINK);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::genericTopicHandler(WINDOW *win, bool mini) {
  std::vector<mrs_msgs::msg::CustomTopic> custom_topic_vec;
  bool avoiding_collision, can_takeoff, null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    custom_topic_vec   = uav_status_.custom_topics;
    avoiding_collision = uav_status_.avoiding_collision;
    can_takeoff        = uav_status_.automatic_start_can_takeoff;
    null_tracker       = uav_status_.null_tracker;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  if (!custom_topic_vec.empty()) {

    for (size_t i = 0; i < custom_topic_vec.size(); i++) {

      wattron(win, COLOR_PAIR(custom_topic_vec[i].topic_color));
      if (mini) {
        printCompressedLimitedString(win, 1 + i, 1, custom_topic_vec[i].topic_name, 4);
        printLimitedDouble(win, 1 + i, 5, "%3.0f", custom_topic_vec[i].topic_hz, 1000);
      } else {
        printLimitedString(win, 1 + i, 1, custom_topic_vec[i].topic_name, 15);
        printLimitedDouble(win, 1 + i, 16, "%5.1f Hz", custom_topic_vec[i].topic_hz, 1000);
      }
      wattroff(win, COLOR_PAIR(custom_topic_vec[i].topic_color));
    }

  } else {

    werase(win);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::nodeStatsHandler(WINDOW *win) {
  mrs_msgs::msg::NodeCpuLoad node_cpu_load_vec;
  double cpu_load_total;
  bool   avoiding_collision, can_takeoff, null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    node_cpu_load_vec  = uav_status_.node_cpu_loads;
    cpu_load_total     = uav_status_.cpu_load_total;
    avoiding_collision = uav_status_.avoiding_collision;
    can_takeoff        = uav_status_.automatic_start_can_takeoff;
    null_tracker       = uav_status_.null_tracker;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  if (!node_cpu_load_vec.node_names.empty()) {
    size_t tmp_num_lines = node_cpu_load_vec.node_names.size();
    if (tmp_num_lines > 9) {
      tmp_num_lines = 9;
    }

    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
    printLimitedString(win, 0, 1, "ROS Node CPU usage", 40);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

    printLimitedDouble(win, 0, 37, "%5.1f", cpu_load_total, 9999);
    printLimitedString(win, 0, 43, "CPU %%", 6);
    for (size_t i = 0; i < tmp_num_lines; i++) {

      printLimitedString(win, 1 + i, 1, node_cpu_load_vec.node_names[i], 42);

      short tmp_color = static_cast<int>(ColorPair::Green);
      if (node_cpu_load_vec.cpu_loads[i] > 99.9) {
        tmp_color = static_cast<int>(ColorPair::Red);
      } else if (node_cpu_load_vec.cpu_loads[i] > 49.9) {
        tmp_color = static_cast<int>(ColorPair::Yellow);
      }

      wattron(win, COLOR_PAIR(tmp_color));
      printLimitedDouble(win, 1 + i, 43, "%5.1f", node_cpu_load_vec.cpu_loads[i], 9999);
      wattroff(win, COLOR_PAIR(tmp_color));
    }

  } else {

    werase(win);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::uavStateHandler(WINDOW *win, bool mini) {
  double avg_rate, color, heading;
  double state_x, state_y, state_z;
  double cmd_x, cmd_y, cmd_z, cmd_hdg;
  std::string odom_frame, main_estimator, horizontal_estimator, vertical_estimator, heading_estimator, agl_estimator;
  double max_flight_z;
  bool   null_tracker, avoiding_collision, can_takeoff;

  {
    std::scoped_lock lock(mutex_status_msg_);
    avg_rate   = uav_status_.odom_hz;
    color      = uav_status_.odom_color;
    heading    = uav_status_.odom_hdg;
    state_x    = uav_status_.odom_x;
    state_y    = uav_status_.odom_y;
    state_z    = uav_status_.odom_z;
    odom_frame = uav_status_.odom_frame;

    cmd_x   = uav_status_.cmd_x;
    cmd_y   = uav_status_.cmd_y;
    cmd_z   = uav_status_.cmd_z;
    cmd_hdg = uav_status_.cmd_hdg;

    uav_status_.odom_estimators.empty() ? main_estimator = "NONE" : main_estimator = uav_status_.odom_estimators[0];

    horizontal_estimator = uav_status_.horizontal_estimator;
    vertical_estimator   = uav_status_.vertical_estimator;
    heading_estimator    = uav_status_.heading_estimator;
    agl_estimator        = uav_status_.agl_estimator;

    max_flight_z       = uav_status_.max_flight_z;
    null_tracker       = uav_status_.null_tracker;
    avoiding_collision = uav_status_.avoiding_collision;
    can_takeoff        = uav_status_.automatic_start_can_takeoff;
  }

  double cerr_x   = std::fabs(state_x - cmd_x);
  double cerr_y   = std::fabs(state_y - cmd_y);
  double cerr_z   = std::fabs(state_z - cmd_z);
  double cerr_hdg = std::fabs(radians::diff(heading, cmd_hdg));

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  if (mini) {
    printLimitedDouble(win, 0, 1, "Odm %3.0f", avg_rate, 1000);

    if (avg_rate == 0) {

      printNoData(win, 0, 1, mini);

    } else {

      printLimitedDouble(win, 1, 1, "%4.0f", state_x, 1000);
      printLimitedDouble(win, 2, 1, "%4.0f", state_y, 1000);
      printLimitedDouble(win, 3, 1, "%4.0f", state_z, 1000);
      printLimitedDouble(win, 4, 1, "%4.1f", heading, 1000);

      printLimitedString(win, 1, 6, main_estimator, 2);
    }
  }

  else {

    printLimitedDouble(win, 0, 12, "Odom %5.1f Hz", avg_rate, 1000);

    if (avg_rate == 0) {

      printNoData(win, 0, 1, mini);

    } else {

      printLimitedDouble(win, 1, 1, "X %7.2f", state_x, 1000);
      printLimitedDouble(win, 2, 1, "Y %7.2f", state_y, 1000);
      printLimitedDouble(win, 3, 1, "Z %7.2f", state_z, 1000);
      printLimitedDouble(win, 4, 1, "hdg %5.2f", heading, 1000);

      if (!null_tracker) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        mvwprintw(win, 5, 1, "C/E");

        if (cerr_x < 0.5) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_x < 1.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 5, "X%1.1f", cerr_x, 10);


        if (cerr_y < 0.5) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_y < 1.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 10, "Y%1.1f", cerr_y, 10);

        if (cerr_z < 0.5) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_z < 1.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 15, "Z%1.1f", cerr_z, 10);

        if (cerr_hdg < 0.2) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_hdg < 0.4) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 20, "H%1.1f", cerr_hdg, 10);

        wattron(win, COLOR_PAIR(color));
      }

      printLimitedString(win, 1, 11, main_estimator, 14);

      switch (estimator_display_counter_) {
      case 0: printLimitedString(win, 2, 11, "hor: " + horizontal_estimator, 14); break;
      case 1: printLimitedString(win, 2, 11, "ver: " + vertical_estimator, 14); break;
      case 2: printLimitedString(win, 2, 11, "hdg: " + heading_estimator, 14); break;
      }


      printLimitedString(win, 4, 11, "ag: " + agl_estimator, 14);

      double dist_to_max_z = max_flight_z - state_z;
      if (dist_to_max_z < 0.0) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        wattron(win, A_BLINK);
      } else if (dist_to_max_z < 0.3) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (dist_to_max_z < 1.0) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      } else {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      }

      printLimitedDouble(win, 3, 11, "Max: %5.1f", max_flight_z, 1000);
      wattron(win, COLOR_PAIR(color));
      wattroff(win, A_BLINK);
    }
  }

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}

void TUI::controlManagerHandler(WINDOW *win, bool mini) {
  int16_t color;
  bool null_tracker, avoiding_collision, can_takeoff;
  double rate;
  std::string curr_controller, curr_tracker, curr_gains, curr_constraints;
  bool callbacks_enabled, rc_mode, have_goal, tracking_trajectory;

  {
    std::scoped_lock lock(mutex_status_msg_);
    rate  = uav_status_.control_manager_diag_hz;
    color = uav_status_.control_manager_diag_color;

    uav_status_.controllers.empty()  ? curr_controller  = "NONE" : curr_controller  = uav_status_.controllers[0];
    uav_status_.trackers.empty()     ? curr_tracker     = "NONE" : curr_tracker     = uav_status_.trackers[0];
    uav_status_.gains.empty()        ? curr_gains       = "NONE" : curr_gains       = uav_status_.gains[0];
    uav_status_.constraints.empty()  ? curr_constraints = "NONE" : curr_constraints = uav_status_.constraints[0];

    callbacks_enabled   = uav_status_.callbacks_enabled;
    rc_mode             = uav_status_.rc_mode;
    have_goal           = uav_status_.have_goal;
    tracking_trajectory = uav_status_.tracking_trajectory;
    null_tracker        = uav_status_.null_tracker;
    avoiding_collision  = uav_status_.avoiding_collision;
    can_takeoff         = uav_status_.automatic_start_can_takeoff;
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, COLOR_PAIR(color));

  if (mini) {
    printLimitedDouble(win, 0, 1, "Ctr %3.0f", rate, 1000);

    if (rate == 0.0) {

      printNoData(win, 0, 1, mini);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 1, 1, "ERR");
      mvwprintw(win, 2, 1, "ERR");
      wattroff(win, COLOR_PAIR(color));

    } else {

      if (curr_controller != "Se3Controller" && curr_controller != "MpcController") {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        printLimitedString(win, 1, 1, curr_controller, 3);
      } else {
        printLimitedString(win, 1, 1, curr_controller, 3);
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        mvwprintw(win, 1, 4, "%s", "/");
      }

      wattron(win, COLOR_PAIR(color));

      if (null_tracker) {
        curr_tracker = "NlT";
      }

      if (curr_tracker != "MpcTracker") {
        if (curr_tracker == "LandoffTracker" && color != static_cast<int>(ColorPair::Red)) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }

        printLimitedString(win, 2, 1, curr_tracker, 3);

      } else {
        printLimitedString(win, 2, 1, curr_tracker, 3);
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        mvwprintw(win, 2, 4, "%s", "/");
        wattron(win, COLOR_PAIR(color));
      }

      printLimitedString(win, 1, 5, curr_gains, 3);
      printLimitedString(win, 2, 5, curr_constraints, 3);
    }
  }

  else {

    printLimitedDouble(win, 0, 1, "Control Manager %5.1f Hz", rate, 1000);

    if (rate == 0.0) {

      printNoData(win, 0, 1, mini);

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 1, 1, "NO_CONTROLLER");
      mvwprintw(win, 2, 1, "NO_TRACKER");
      wattroff(win, COLOR_PAIR(color));

    } else {
      if (curr_controller != "Se3Controller" && curr_controller != "MpcController") {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }
      printLimitedString(win, 1, 1, curr_controller, 13);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
      printLimitedString(win, 1, 1 + std::min(int(curr_controller.length()), 13), "/" + curr_gains, 10);
      wattron(win, COLOR_PAIR(color));

      if (null_tracker) {
        curr_tracker = "NullTracker";
      }

      if (curr_tracker != "MpcTracker") {
        if (curr_tracker == "LandoffTracker" && color != static_cast<int>(ColorPair::Red)) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
      }

      printLimitedString(win, 2, 1, curr_tracker, 13);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
      printLimitedString(win, 2, 1 + std::min(int(curr_tracker.length()), 13), "/" + curr_constraints, 8);
      wattron(win, COLOR_PAIR(color));
    }

    if (rc_mode) {
      wattron(win, A_BLINK);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 1, 18, "RC_MODE");
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      wattroff(win, A_BLINK);

    } else if (!callbacks_enabled) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 1, 20, "NO_CB");
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }

    if (tracking_trajectory) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      mvwprintw(win, 2, 21, "TRAJ");
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

    } else if (have_goal) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      mvwprintw(win, 2, 21, "GOTO");
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

    } else {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      mvwprintw(win, 2, 21, "IDLE");
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
    }
  }

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::hwApiStateHandler(WINDOW *win, bool mini) {
  int16_t color;
  double hw_api_rate, state_rate, cmd_rate, battery_rate;
  bool gnss_ok, armed;
  std::string mode;
  double battery_volt, battery_curr, battery_wh_drained;
  double thrust, mass_estimate, mass_set, gnss_qual, mag_norm, mag_norm_rate;
  bool avoiding_collision, can_takeoff, null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    color              = uav_status_.hw_api_color;
    hw_api_rate        = uav_status_.hw_api_hz;
    state_rate         = uav_status_.hw_api_state_hz;
    cmd_rate           = uav_status_.hw_api_cmd_hz;
    battery_rate       = uav_status_.hw_api_battery_hz;
    gnss_ok            = uav_status_.hw_api_gnss_ok;
    armed              = uav_status_.hw_api_armed;
    mode               = uav_status_.hw_api_mode;
    battery_volt       = uav_status_.battery_volt;
    battery_curr       = uav_status_.battery_curr;
    battery_wh_drained = uav_status_.battery_wh_drained;
    thrust             = uav_status_.thrust;
    mass_estimate      = uav_status_.mass_estimate;
    mass_set           = uav_status_.mass_set;
    gnss_qual          = uav_status_.hw_api_gnss_qual;
    mag_norm           = uav_status_.mag_norm;
    mag_norm_rate      = uav_status_.mag_norm_hz;
    avoiding_collision = uav_status_.avoiding_collision;
    can_takeoff        = uav_status_.automatic_start_can_takeoff;
    null_tracker       = uav_status_.null_tracker;
  }

  std::string tmp_string;

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  if (mini) {
    printLimitedDouble(win, 0, 1, "Mav %3.0f", hw_api_rate, 1000);
    wattroff(win, COLOR_PAIR(color));

    if (hw_api_rate == 0) {
      printNoData(win, 0, 1, mini);
    }

    if (state_rate == 0) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 1, "ERR", 3);
      printLimitedString(win, 2, 1, "ERR", 3);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      if (armed) {
        tmp_string = "ARM";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      } else {
        tmp_string = "DIS";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 1, 1, tmp_string, 15);
      wattron(win, COLOR_PAIR(color));

      if (mode != "OFFBOARD") {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 2, 1, mode, 3);
      wattron(win, COLOR_PAIR(color));
    }

    if (battery_rate == 0) {

      printLimitedString(win, 3, 1, "ERR", 3);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      (battery_volt > 17.0) ? (battery_volt = battery_volt / 6) : (battery_volt = battery_volt / 4);

      if (battery_volt < 3.6) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (battery_volt < 3.7 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedString(win, 3, 1, "Bat", 3);
    }


    if (cmd_rate == 0) {

      printLimitedString(win, 3, 5, "ERR", 3);

    } else {

      if (thrust > 0.75) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (thrust > 0.65 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 3, 5, ".%2.0f", thrust * 100, 100);
      wattron(win, COLOR_PAIR(color));

      color            = static_cast<int>(ColorPair::Green);
      double mass_diff = std::fabs(mass_estimate - mass_set) / mass_set;

      if (mass_diff > 0.3) {

        color = static_cast<int>(ColorPair::Red);

      } else if (mass_diff > 0.2) {

        color = static_cast<int>(ColorPair::Yellow);
      }

      printLimitedDouble(win, 4, 1, "%4.1f kg", mass_estimate, 99.99);
    }

    if (!gnss_ok) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 5, "GPS", 6);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      printLimitedString(win, 1, 5, "GPS", 6);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      color = static_cast<int>(ColorPair::Red);

      if (gnss_qual < 5.0) {
        color = static_cast<int>(ColorPair::Green);
      } else if (gnss_qual < 10.0) {
        color = static_cast<int>(ColorPair::Yellow);
      }

      wattron(win, COLOR_PAIR(color));

      if (gnss_qual < 10.0) {
        printLimitedDouble(win, 2, 5, "%3.1f", gnss_qual, 9.9);
      } else {
        printLimitedString(win, 2, 5, ">10", 3);
      }
      wattroff(win, COLOR_PAIR(color));
    }

  }

  else {

    printLimitedDouble(win, 0, 9, "HW Api %5.1f Hz", hw_api_rate, 1000);
    wattroff(win, COLOR_PAIR(color));

    if (hw_api_rate == 0) {

      printNoData(win, 0, 1, mini);
    }

    if (state_rate == 0) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 1, "State: ", 15);
      printNoData(win, 1, 9, mini);
      printLimitedString(win, 2, 1, "Mode: ", 15);
      printNoData(win, 1, 9, mini);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      if (armed) {
        tmp_string = "ARMED";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      } else {
        tmp_string = "DISARMED";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 1, 1, "State: " + tmp_string, 15);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (mode != "OFFBOARD") {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 2, 1, "Mode:  " + mode, 15);
      wattron(win, COLOR_PAIR(color));
    }

    if (battery_rate == 0) {

      printNoData(win, 4, 1, "Batt:  ", mini);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      (battery_volt > 17.0) ? (battery_volt = battery_volt / 6) : (battery_volt = battery_volt / 4);

      if (battery_volt < 3.6) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (battery_volt < 3.7 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 4, 1, "%4.2fV ", battery_volt, 10);
      printLimitedDouble(win, 4, 8, "%5.2fA", battery_curr, 100);
      printLimitedDouble(win, 4, 15, " %4.1f Wh", battery_wh_drained, 100);
    }

    if (mag_norm_rate == 0) {

      printNoData(win, 3, 1, "Mag:  ", mini);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (mag_norm > 0.9 || mag_norm < 0.25) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (mag_norm > 0.65) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 3, 1, "Mag: %4.2f", mag_norm, 9.99);
    }

    if (cmd_rate == 0) {

      printNoData(win, 5, 1, "Thrst: ", mini);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (thrust > 0.75) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (thrust > 0.65 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 5, 1, "Thrst: %4.2f", thrust, 1.01);
      wattron(win, COLOR_PAIR(color));

      color            = static_cast<int>(ColorPair::Green);
      double mass_diff = std::fabs(mass_estimate - mass_set) / mass_set;

      if (mass_diff > 0.3) {

        color = static_cast<int>(ColorPair::Red);

      } else if (mass_diff > 0.2) {

        color = static_cast<int>(ColorPair::Yellow);
      }

      if (mass_set > 10.0 || mass_estimate > 10.0) {

        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        printLimitedDouble(win, 5, 13, "%.1f/", mass_set, 99.99);
        wattron(win, COLOR_PAIR(color));
        printLimitedDouble(win, 5, 18, "%.1f", mass_estimate, 99.99);
        printLimitedString(win, 5, 22, "kg", 2);

      } else {

        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        printLimitedDouble(win, 5, 15, "%.1f/", mass_set, 99.99);
        wattron(win, COLOR_PAIR(color));
        printLimitedDouble(win, 5, 19, "%.1f", mass_estimate, 99.99);
        printLimitedString(win, 5, 22, "kg", 2);
      }
    }

    if (!gnss_ok) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 18, "NO_GPS", 6);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      printLimitedString(win, 1, 18, "GPS_OK", 6);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      color = static_cast<int>(ColorPair::Red);

      if (gnss_qual < 5.0) {
        color = static_cast<int>(ColorPair::Green);
      } else if (gnss_qual < 10.0) {
        color = static_cast<int>(ColorPair::Yellow);
      }

      wattron(win, COLOR_PAIR(color));
      printLimitedDouble(win, 2, 17, "Q: %4.1f", gnss_qual, 99.9);
      wattroff(win, COLOR_PAIR(color));
    }
  }

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}

void TUI::topLineHandler(WINDOW *win, bool mini) {
  werase(win);
  int secs_flown;

  {
    std::scoped_lock lock(mutex_status_msg_);
    secs_flown = uav_status_.secs_flown;
  }

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  printLimitedInt(win, 0, 0, "ToF: %i", secs_flown, 1000);

  std::string uav_name, uav_type;
  bool collision_avoidance_enabled, avoiding_collision;
  uint16_t num_other_uavs;

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_name                    = uav_status_.uav_name;
    uav_type                    = uav_status_.uav_type;
    collision_avoidance_enabled = uav_status_.collision_avoidance_enabled;
    avoiding_collision          = uav_status_.avoiding_collision;
    num_other_uavs              = uav_status_.num_other_uavs;
  }

  double tmp_time       = (clock_->now() - last_time_got_data_).seconds();
  double tmp_short_time = (clock_->now() - last_time_got_short_data_).seconds();

  if (tmp_short_time < 3.0) {
    have_short_data_ = true;
  } else {
    have_short_data_ = false;
  }

  if (tmp_short_time >= 99.9) {
    tmp_short_time = 99.9;
  }

  if (tmp_time > 3.0 && have_data_) {
    have_data_ = false;
    _light_    = setupColors(have_data_, _colorscheme_, _colorblind_mode_);
  }

  if (tmp_time < 3.0 && !have_data_) {
    have_data_ = true;
    _light_    = setupColors(have_data_, _colorscheme_, _colorblind_mode_);
  }

  if (tmp_time >= 99.9) {
    tmp_time = 99.9;
  }

  mvwprintw(win, 0, 10, " %s %s ", uav_name.c_str(), uav_type.c_str());

  if (!mini) {
    if (collision_avoidance_enabled) {
      if (avoiding_collision) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        wattron(win, A_BLINK);
        mvwprintw(win, 0, 26, "!! AVOIDING COLLISION !!");
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        wattroff(win, A_BLINK);
      } else {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        mvwprintw(win, 0, 26, "COL AVOID ENABLED,");
        if (num_other_uavs == 0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        mvwprintw(win, 0, 45, "UAVs: ");
        printLimitedInt(win, 0, 51, "%i", num_other_uavs, 100);
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }
    } else {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 0, 26, "COL AVOID DISABLED");
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }
  } else {

    if (collision_avoidance_enabled) {

      if (avoiding_collision) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        wattron(win, A_BLINK);
        mvwprintw(win, 0, 22, "!AVOIDING!");
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        wattroff(win, A_BLINK);
      } else {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        mvwprintw(win, 0, 27, "C/A");
        if (num_other_uavs == 0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedInt(win, 0, 31, "%i", num_other_uavs, 100);
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }
    } else {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 0, 27, "C/A");
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }
  }

  if (!have_data_) {
    wattron(win, A_BLINK);
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::AlwaysRed)));
    mvwprintw(win, 0, 0, "!NO MSGS!");
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::AlwaysRed)));
    wattroff(win, A_BLINK);
  }

  int mins = secs_flown / 60;
  int secs = secs_flown % 60;

  mvwprintw(win, 0, 0, "ToF: %i:%02i", mins, secs);
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}


} // namespace mrs_uav_status::tui
