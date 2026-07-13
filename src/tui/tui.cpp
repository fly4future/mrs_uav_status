#include <mrs_uav_status/tui/tui.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <mrs_lib/geometry/cyclic.h>
#include <mrs_msgs/msg/reference_stamped.hpp>

namespace mrs_uav_status::tui
{

using radians = mrs_lib::geometry::radians;


TUI::TUI(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_sc, const TUI::TUIParams &params)
    : node_(node), clock_(node->get_clock()), params_(params) {

  _light_ = (params_.colorscheme.find("COLORSCHEME_LIGHT") != std::string::npos);

  setupPanes();

  last_time_got_data_       = rclcpp::Time(0, 0, clock_->get_clock_type());
  bottom_window_clear_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  last_time_got_general_robot_info_       = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_time_got_collision_avoidance_info_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_time_got_uav_info_                 = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_time_got_system_health_info_       = rclcpp::Time(0, 0, clock_->get_clock_type());

  goto_double_vec_   = params_.goto_values;
  service_input_vec_ = params_.service_list;

  for (const auto &service_input : service_input_vec_) {
    std::vector<std::string> results = utils::splitByChar(service_input, ' ');

    if (results.size() < 2) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid service entry: '%s'. Each entry must contain at least a service name and a display name, separated by a space.",
                   service_input.c_str());
      continue;
    }

    for (unsigned long j = 2; j < results.size(); j++) {
      results[1] = results[1] + " " + results[j];
    }

    std::string service_name;

    if (results[0].at(0) == '/') {
      service_name = results[0];
    } else {
      service_name = "/" + params_.uav_name + "/" + results[0];
    }

    auto service_display_name = results[1];
    service_entries_.emplace_back(service_display_name, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, service_name, cbkgrp_sc));
  }

  sc_goto_reference_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/goto_reference_out", cbkgrp_sc);
  sc_velocity_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv>(node_, "~/velocity_reference_out", cbkgrp_sc);
  sc_set_constraints_    = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_constraints_out", cbkgrp_sc);
  sc_set_gains_          = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_gains_out", cbkgrp_sc);
  sc_set_controller_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_controller_out", cbkgrp_sc);
  sc_set_tracker_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_tracker_out", cbkgrp_sc);
  sc_set_estimator_      = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_estimator_out", cbkgrp_sc);
  sc_hover_              = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/hover_out", cbkgrp_sc);
  sc_toggle_output_      = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/toggle_output_out", cbkgrp_sc);

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);
}

void TUI::onGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  last_general_robot_info_          = msg;
  last_time_got_data_               = clock_->now();
  last_time_got_general_robot_info_ = last_time_got_data_;
}

void TUI::onStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  last_state_estimation_info_ = msg;
  last_time_got_data_         = clock_->now();
}

void TUI::onControlInfo(const mrs_msgs::msg::ControlInfo &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  last_control_info_  = msg;
  last_time_got_data_ = clock_->now();
}

void TUI::onCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  last_collision_avoidance_info_          = msg;
  last_time_got_data_                     = clock_->now();
  last_time_got_collision_avoidance_info_ = last_time_got_data_;
}

void TUI::onUavInfo(const mrs_msgs::msg::UavInfo &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  last_uav_info_          = msg;
  last_time_got_data_     = clock_->now();
  last_time_got_uav_info_ = last_time_got_data_;
}

void TUI::onSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  last_system_health_info_          = msg;
  last_time_got_data_               = clock_->now();
  last_time_got_system_health_info_ = last_time_got_data_;
}

void TUI::onUavState(const mrs_msgs::msg::State &msg) {
  std::scoped_lock lock(mutex_status_msg_);
  last_uav_state_     = msg;
  last_time_got_data_ = clock_->now();
}

void TUI::onString(const std_msgs::msg::String &msg) {
  // Parse leading flags ("-id <key>" optional dedupe key, "-p" mark persistent),
  // rejoin remaining tokens as the display text, then dedupe-or-append in
  // string_info_vec_. Mirrors the legacy data_acquisition.cpp::callbackString.
  std::stringstream                  ss(msg.data);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string>           tokens(begin, end);
  if (tokens.empty()) {
    return;
  }

  std::string id;
  bool        persistent = false;
  bool        flags_done = false;
  size_t      i          = 0;
  while (!flags_done && i < tokens.size()) {
    if (tokens[i] == "-id" && i + 1 < tokens.size()) {
      id = tokens[i + 1];
      tokens.erase(tokens.begin() + i, tokens.begin() + i + 2);
    } else if (tokens[i] == "-p") {
      persistent = true;
      tokens.erase(tokens.begin() + i);
    } else if (!tokens[i].empty() && tokens[i].front() != '-') {
      flags_done = true;
    } else {
      ++i;
    }
  }

  std::string display;
  for (size_t k = 0; k < tokens.size(); ++k) {
    if (k > 0) {
      display += ' ';
    }
    display += tokens[k];
  }

  std::scoped_lock   lock(mutex_status_msg_);
  const rclcpp::Time now = clock_->now();
  // Dedupe by id (the legacy publisher_name was already lost from ROS1 → always empty).
  for (auto &entry : string_info_vec_) {
    if (entry.id == id) {
      entry.display_string = display;
      entry.persistent     = persistent;
      entry.last_time      = now;
      return;
    }
  }
  string_info_vec_.emplace_back(now, std::string{}, display, id, persistent);
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

void TUI::setupWindows() {

  std::string command = "tmux display-message -p '#S'";
  session_name_       = utils::callTerminal(command.c_str());
  session_name_.erase(std::remove(session_name_.begin(), session_name_.end(), '\n'), session_name_.end());

  if (params_.start_minimized) {
    control_manager_window_.reset(newwin(4, 9, 1, 1));
    uav_state_window_.reset(newwin(6, 9, 5, 1));
    top_bar_window_.reset(newwin(1, 140, 0, 1));
    general_info_window_.reset(newwin(4, 9, 1, 10));
    hw_api_state_window_.reset(newwin(6, 9, 5, 10));
    debug_window_.reset(newwin(terminal_lines_ - 15, terminal_cols_ - 1, 13, 1));
    pane_window_.reset();
    bottom_window_.reset(newwin(1, 120, 11, 1));

  } else {

    uav_state_window_.reset(newwin(7, 26, 5, 1));
    control_manager_window_.reset(newwin(4, 26, 1, 1));
    hw_api_state_window_.reset(newwin(7, 25, 5, 27));
    general_info_window_.reset(newwin(4, 25, 1, 27));
    top_bar_window_.reset(newwin(1, 140, 0, 1));
    bottom_window_.reset(newwin(1, 120, 12, 1));
    pane_window_.reset(newwin(11, 82, 1, 52));
    const int debug_height = std::max(3, terminal_lines_ - 13);
    debug_window_.reset(newwin(debug_height, terminal_cols_ - 1, 13, 1));
    const int half_lines = std::max(1, (debug_height - 2) / 2);
    sub_tmux_window_1_.reset(derwin(debug_window_.get(), half_lines, terminal_cols_ - 3, 1, 1));
    sub_tmux_window_2_.reset(derwin(debug_window_.get(), half_lines, terminal_cols_ - 3, half_lines + 2, 1));
  }

  clear();
  refresh();
  _light_ = tui::setupColors(have_data_, params_.colorscheme, params_.colorblind_mode);
}

bool TUI::resize() {
  if (!updateTermSize()) {
    return false;
  }
  if (terminal_cols_ > 30) {
    resize_term(terminal_lines_, terminal_cols_);
    setupWindows();
    return true;
  }
  return false;
}

void TUI::toggleMini() {
  params_.start_minimized = !params_.start_minimized;
}

void TUI::toggleHelp() {
  help_active_ = !help_active_;
}

void TUI::setRemoteMode(bool in_remote_mode) {
  in_remote_mode_ = in_remote_mode;
}

bool TUI::isFlyingNormally() {
  std::scoped_lock lock(mutex_status_msg_);
  return last_control_info_.flying_normally;
}

void TUI::refreshTopBar() {
  wnoutrefresh(top_bar_window_.get());
}

void TUI::refreshBottomWindow() {
  wnoutrefresh(bottom_window_.get());
}

void TUI::refreshAfterMenu() {
  wnoutrefresh(debug_window_.get());
  wnoutrefresh(bottom_window_.get());
}

void TUI::renderFast() {
  topLineHandler();
  renderTmuxOrHelp();
  uavStateHandler();
}

void TUI::renderSlow() {
  tickSlowCounter();
  hwApiStateHandler();
  controlManagerHandler();
  paneHandler();
  generalInfoHandler();
}

void TUI::generalInfoHandler() {
  WINDOW *win = general_info_window_.get();
  werase(win);
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  wattroff(win, A_STANDOUT);

  bool   avoiding_collision, bumper_active, can_takeoff, null_tracker;
  double cpu_load, cpu_ghz, free_ram, total_ram;
  int    free_hdd;
  bool   have_system_health_info;
  {
    std::scoped_lock lock(mutex_status_msg_);
    const auto      &oc     = last_system_health_info_.onboard_computer_info;
    avoiding_collision      = last_collision_avoidance_info_.avoiding_collision;
    bumper_active           = last_collision_avoidance_info_.bumper_active;
    can_takeoff             = last_general_robot_info_.ready_to_start;
    null_tracker            = (last_control_info_.active_tracker == "NullTracker");
    cpu_load                = oc.cpu_load;
    cpu_ghz                 = oc.cpu_ghz;
    free_ram                = oc.free_ram;
    total_ram               = oc.total_ram;
    free_hdd                = oc.free_hdd;
    have_system_health_info = last_time_got_system_health_info_.nanoseconds() != 0;
  }

  printBox(win, avoiding_collision, bumper_active, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  if (!have_system_health_info) {
    // Fields default to -1/-1.0 and would otherwise render as a healthy-looking 0% CPU/RAM.
    printNoData(win, 1, 1, params_.start_minimized);
  } else {
    printCpuLoad(win, cpu_load, params_.start_minimized);
    printMemLoad(win, free_ram, total_ram, params_.start_minimized);
    if (!params_.start_minimized) {
      printCpuFreq(win, cpu_ghz);
    }
    printDiskSpace(win, free_hdd, last_gigas_, params_.start_minimized);
    last_gigas_ = free_hdd;
  }

  wnoutrefresh(win);
}

void TUI::renderStringsGnssPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<std::string> string_vector;
  uint8_t                  gnss_fix_type      = 0;
  uint8_t                  gnss_num_sats      = 0;
  double                   gnss_pos_acc       = 100.0;
  double                   gnss_status_rate   = 0.0;
  bool                     gnss_status_msg_ok = false;

  {
    std::scoped_lock lock(mutex_status_msg_);
    if (const auto *gnss = utils::findSensor(last_system_health_info_.available_sensors, mrs_msgs::msg::SensorStatus::TYPE_GNSS); gnss) {
      const std::string fix_type_raw = utils::lookupDetail(gnss->details, "fix_type");
      gnss_status_msg_ok             = (fix_type_raw != "nan");
      gnss_fix_type                  = static_cast<uint8_t>(utils::parseLongOr(fix_type_raw, 0));
      gnss_num_sats                  = static_cast<uint8_t>(utils::parseLongOr(utils::lookupDetail(gnss->details, "num_satellites"), 0));
      gnss_pos_acc                   = utils::parseDoubleOr(utils::lookupDetail(gnss->details, "position_accuracy"), 100.0);
      gnss_status_rate               = gnss->rate;
    }

    // Custom strings: evict stale (>10 s, unless persistent), collect the rest.
    const rclcpp::Time now = clock_->now();
    for (auto it = string_info_vec_.begin(); it != string_info_vec_.end();) {
      if (!it->persistent && (now - it->last_time).seconds() > 10.0) {
        it = string_info_vec_.erase(it);
      } else {
        string_vector.push_back(it->display_string);
        ++it;
      }
    }
  }

  if (gnss_status_rate > 0.0 && gnss_status_msg_ok) {
    std::string fix_string;
    if (gnss_fix_type < 1 || gnss_fix_type >= 8) {
      fix_string += "-r ";
    }
    fix_string += "Fix Type: ";

    switch (gnss_fix_type) {
    case 0:
      fix_string += "NO GNSS";
      break;
    case 1:
      fix_string += "NO FIX";
      break;
    case 2:
      fix_string += "2D FIX";
      break;
    case 3:
      fix_string += "3D FIX";
      break;
    case 4:
      fix_string += "3D SBAS FIX";
      break;
    case 5:
      fix_string += "RTK FLOAT";
      break;
    case 6:
      fix_string += "RTK FIX (INT)";
      break;
    case 7:
      fix_string += "STATIC - BASESTATION";
      break;
    case 8:
      fix_string += "PPP 3D FIX";
      break;
    default:
      fix_string += "UNKNOWN";
      break;
    }

    std::string gnss_acc_string;
    if (gnss_pos_acc >= 100.0) {
      gnss_acc_string = "N/A";
    } else {
      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << gnss_pos_acc;
      gnss_acc_string = stream.str();
    }
    string_vector.push_back(fix_string);
    string_vector.push_back("Num sats: " + std::to_string(gnss_num_sats) + " Acc: " + gnss_acc_string + " m");
  }

  if (string_vector.empty()) {
    string_vector.push_back("-y no GNSS / strings data");
  }

  constexpr int max_rows = 9;
  for (const auto &raw : string_vector) {
    if (row > max_rows) {
      break;
    }

    int         tmp_color = static_cast<int>(ColorPair::Normal);
    bool        blink     = false;
    std::string display   = raw;

    // In-band colour tag: leading "-R"/"-r"/"-Y"/"-y"/"-G"/"-g" (uppercase blinks).
    if (display.size() >= 3 && display[0] == '-') {
      switch (display[1]) {
      case 'R':
        blink = true;
        [[fallthrough]];
      case 'r':
        tmp_color = static_cast<int>(ColorPair::Red);
        break;
      case 'Y':
        blink = true;
        [[fallthrough]];
      case 'y':
        tmp_color = static_cast<int>(ColorPair::Yellow);
        break;
      case 'G':
        blink = true;
        [[fallthrough]];
      case 'g':
        tmp_color = static_cast<int>(ColorPair::Green);
        break;
      }
      if (tmp_color != static_cast<int>(ColorPair::Normal)) {
        display.erase(0, 3);
      }
    }

    if (blink) {
      wattron(win, A_BLINK);
    }
    wattron(win, COLOR_PAIR(tmp_color));
    printLimitedString(win, row++, 1, display, 80);
    wattroff(win, COLOR_PAIR(tmp_color));
    wattroff(win, A_BLINK);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::setupPanes() {
  panes_.clear();

  // Default pane: available sensors (dynamic, plugin-driven) — name/rate/status
  panes_.push_back({"Sensors", [this](WINDOW *win) { renderSensorsPane(win); }, nullptr});
  // ROS per-node CPU usage (was its own top-right box)
  panes_.push_back({"ROS Node CPU", [this](WINDOW *win) { renderNodeCpuPane(win); }, nullptr});
  // GNSS fix + custom display strings
  panes_.push_back({"GNSS & strings", [this](WINDOW *win) { renderStringsGnssPane(win); }, nullptr});
  // Problems + errors. Auto-focused when either becomes non-empty
  panes_.push_back({"Problems & errors", [this](WINDOW *win) { renderProblemsPane(win); },
                    [this]() {
                      std::scoped_lock lock(mutex_status_msg_);
                      return !last_general_robot_info_.problems_preventing_start.empty() || !last_general_robot_info_.errors.empty();
                    }});

  // To add a pane push another Pane with a
  // title + render lambda; cycling, the title, and auto-focus pick it up.

  pane_focus_prev_.assign(panes_.size(), false);
  pane_idx_ = 0;
}

void TUI::cyclePanes() {
  if (panes_.empty()) {
    return;
  }
  pane_idx_ = (pane_idx_ + 1) % panes_.size();
}

void TUI::selectPane(std::size_t idx) {
  if (idx < panes_.size()) {
    pane_idx_ = idx;
  }
}

void TUI::paneHandler() {
  WINDOW *win = pane_window_.get();
  if (!win || panes_.empty()) {
    return;
  }

  // auto-focus: when a pane wants_focus switching to it
  for (std::size_t i = 0; i < panes_.size(); ++i) {
    const bool wants = panes_[i].wants_focus && panes_[i].wants_focus();
    if (wants && !pane_focus_prev_[i]) {
      pane_idx_ = i;
    }
    pane_focus_prev_[i] = wants;
  }

  panes_[pane_idx_].render(win);
}

// Shared chrome for every pane: clears the window, draws the box, and writes
// the "<p>reset: <name> (i/N)" label into the top border (so all interior rows
// stay available for content). Returns the first usable content row (1).
int TUI::drawPaneChrome(WINDOW *win) {
  bool avoiding_collision, bumper_active, can_takeoff, null_tracker;
  {
    std::scoped_lock lock(mutex_status_msg_);
    avoiding_collision = last_collision_avoidance_info_.avoiding_collision;
    bumper_active      = last_collision_avoidance_info_.bumper_active;
    can_takeoff        = last_general_robot_info_.ready_to_start;
    null_tracker       = (last_control_info_.active_tracker == "NullTracker");
  }

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, bumper_active, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  // Tab bar in the top border (row 0): numbered tabs. The active tab shows its
  // number + name in brackets (green); the others show just their number in red
  // to signal they're switchable.
  int x = 2;
  for (std::size_t i = 0; i < panes_.size(); ++i) {
    const bool        active = (i == pane_idx_);
    const std::string num    = std::to_string(i + 1);

    if (active) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      const std::string tab = num + " [" + panes_[i].title + "]";
      mvwaddstr(win, 0, x, tab.c_str());
      x += static_cast<int>(tab.size());
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
    } else {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwaddstr(win, 0, x, num.c_str());
      x += static_cast<int>(num.size());
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }
    x += 1; // single-space gap between tabs
  }

  return 1;
}

void TUI::renderProblemsPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<std::string> problems;
  std::vector<std::string> errors;
  bool                     have_general_robot_info;
  {
    std::scoped_lock lock(mutex_status_msg_);
    problems                = last_general_robot_info_.problems_preventing_start;
    errors                  = last_general_robot_info_.errors;
    have_general_robot_info = last_time_got_general_robot_info_.nanoseconds() != 0;
  }

  // Empty vectors are indistinguishable from "genuinely zero problems" without this check.
  if (!have_general_robot_info) {
    printNoData(win, row, 1, false);
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  constexpr int max_rows   = 9;
  constexpr int text_width = 80;

  const auto problems_color = problems.empty() ? ColorPair::Green : ColorPair::Red;
  wattron(win, COLOR_PAIR(static_cast<int>(problems_color)));
  printLimitedString(win, row++, 1, "Problems: " + std::to_string(problems.size()), text_width);
  wattroff(win, COLOR_PAIR(static_cast<int>(problems_color)));

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  for (const auto &p : problems) {
    if (row > max_rows) {
      break;
    }
    printLimitedString(win, row++, 1, "- " + p, text_width);
  }
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  if (row <= max_rows) {
    ++row; // blank separator
  }

  if (row <= max_rows) {
    const auto errors_color = errors.empty() ? ColorPair::Green : ColorPair::Red;
    wattron(win, COLOR_PAIR(static_cast<int>(errors_color)));
    printLimitedString(win, row++, 1, "Errors: " + std::to_string(errors.size()), text_width);
    wattroff(win, COLOR_PAIR(static_cast<int>(errors_color)));
  }

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  for (const auto &e : errors) {
    if (row > max_rows) {
      break;
    }
    printLimitedString(win, row++, 1, "- " + e, text_width);
  }
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::renderSensorsPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<mrs_msgs::msg::SensorStatus> sensors;
  {
    std::scoped_lock lock(mutex_status_msg_);
    sensors = last_system_health_info_.available_sensors;
  }

  // Column header.
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 1, "sensor", 36);
  printLimitedString(win, row, 42, "rate", 8);
  printLimitedString(win, row, 55, "status", 12);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  ++row;

  constexpr int max_rows = 9;
  for (const auto &s : sensors) {
    if (row > max_rows) {
      break;
    }

    printLimitedString(win, row, 1, s.name, 36);
    printLimitedDouble(win, row, 40, "%6.1f", s.rate, 100000);

    // SensorStatus.level: 0=OK, 1=WARN, 2=ERROR, 3=STALE.
    int         color = static_cast<int>(ColorPair::Normal);
    std::string label = "?";
    switch (s.level) {
    case 0:
      color = static_cast<int>(ColorPair::Green);
      label = "OK";
      break;
    case 1:
      color = static_cast<int>(ColorPair::Yellow);
      label = "WARN";
      break;
    case 2:
      color = static_cast<int>(ColorPair::Red);
      label = "ERROR";
      break;
    case 3:
      color = static_cast<int>(ColorPair::Yellow);
      label = "STALE";
      break;
    default:
      break;
    }
    wattron(win, COLOR_PAIR(color));
    printLimitedString(win, row, 55, label, 12);
    wattroff(win, COLOR_PAIR(color));
    ++row;
  }

  if (sensors.empty()) {
    printLimitedString(win, row, 1, "no sensors reported", 40);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::renderNodeCpuPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<mrs_msgs::msg::CpuLoad> node_cpu_loads;
  {
    std::scoped_lock lock(mutex_status_msg_);
    node_cpu_loads = last_system_health_info_.onboard_computer_info.node_cpu_loads;
  }

  // Aggregate (single-core %) total — OnboardComputerInfo doesn't expose it.
  double cpu_load_total = 0.0;
  for (const auto &n : node_cpu_loads) {
    cpu_load_total += n.cpu_load;
  }

  constexpr int max_node_row = 9;

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 42, "Total CPU", 20);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedDouble(win, row, 55, "%5.1f", cpu_load_total, 9999);
  printLimitedString(win, row, 61, "%", 5);
  ++row;

  mvwhline(win, row, 55, ACS_HLINE, 7);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 1, "node", 36);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  ++row;

  // Sort by CPU load descending, then name ascending for tie-breaking.
  std::sort(node_cpu_loads.begin(), node_cpu_loads.end(),
            [](const auto &a, const auto &b) { return (a.cpu_load > b.cpu_load) || ((a.cpu_load == b.cpu_load) && (a.node_name < b.node_name)); });

  for (const auto &n : node_cpu_loads) {
    if (row > max_node_row) {
      break;
    }
    printLimitedString(win, row, 1, n.node_name, 36);

    short tmp_color = static_cast<int>(ColorPair::Green);
    if (n.cpu_load > 99.9) {
      tmp_color = static_cast<int>(ColorPair::Red);
    } else if (n.cpu_load > 49.9) {
      tmp_color = static_cast<int>(ColorPair::Yellow);
    }
    wattron(win, COLOR_PAIR(tmp_color));
    printLimitedDouble(win, row, 55, "%5.1f", n.cpu_load, 9999);
    wattroff(win, COLOR_PAIR(tmp_color));
    ++row;
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

void TUI::uavStateHandler() {
  WINDOW     *win = uav_state_window_.get();
  double      avg_rate, color, heading;
  double      state_x, state_y, state_z;
  double      cmd_x, cmd_y, cmd_z, cmd_hdg;
  std::string odom_frame, main_estimator, horizontal_estimator, vertical_estimator, heading_estimator, agl_estimator;
  double      max_flight_z;
  bool        null_tracker, have_control_info, avoiding_collision, bumper_active, can_takeoff;

  {
    std::scoped_lock lock(mutex_status_msg_);
    const auto      &est = last_state_estimation_info_;
    avg_rate             = last_system_health_info_.state_estimation_rate;
    heading              = est.local_pose.heading;
    state_x              = est.local_pose.position.x;
    state_y              = est.local_pose.position.y;
    state_z              = est.local_pose.position.z;
    odom_frame           = est.header.frame_id;

    cmd_x   = last_control_info_.cmd_pose.position.x;
    cmd_y   = last_control_info_.cmd_pose.position.y;
    cmd_z   = last_control_info_.cmd_pose.position.z;
    cmd_hdg = last_control_info_.cmd_pose.heading;

    main_estimator       = est.current_estimator.empty() ? std::string("NONE") : est.current_estimator;
    horizontal_estimator = est.horizontal_estimator;
    vertical_estimator   = est.vertical_estimator;
    heading_estimator    = est.heading_estimator;
    agl_estimator        = est.agl_estimator;

    max_flight_z = est.max_flight_z;
    null_tracker = (last_control_info_.active_tracker == "NullTracker");
    // "unknown" means no TrackerCommand yet, so cmd_pose is still (0,0,0).
    have_control_info  = (last_control_info_.active_tracker != "unknown");
    avoiding_collision = last_collision_avoidance_info_.avoiding_collision;
    bumper_active      = last_collision_avoidance_info_.bumper_active;
    can_takeoff        = last_general_robot_info_.ready_to_start;
  }
  // Nominal MRS estimation rate is 100 Hz; threshold the color band off that.
  color = rateColor(avg_rate, 100.0);

  double cerr_x   = std::fabs(state_x - cmd_x);
  double cerr_y   = std::fabs(state_y - cmd_y);
  double cerr_z   = std::fabs(state_z - cmd_z);
  double cerr_hdg = std::fabs(radians::diff(heading, cmd_hdg));

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, bumper_active, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  if (params_.start_minimized) {
    printLimitedDouble(win, 0, 1, "Odm %3.0f", avg_rate, 1000);

    if (avg_rate == 0) {

      printNoData(win, 0, 1, params_.start_minimized);

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

      printNoData(win, 0, 1, params_.start_minimized);

    } else {

      printLimitedDouble(win, 1, 1, "X %7.2f", state_x, 1000);
      printLimitedDouble(win, 2, 1, "Y %7.2f", state_y, 1000);
      printLimitedDouble(win, 3, 1, "Z %7.2f", state_z, 1000);
      printLimitedDouble(win, 4, 1, "hdg %5.2f", heading, 1000);

      if (!null_tracker && have_control_info) {
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
      case 0:
        printLimitedString(win, 2, 11, "hor: " + horizontal_estimator, 14);
        break;
      case 1:
        printLimitedString(win, 2, 11, "ver: " + vertical_estimator, 14);
        break;
      case 2:
        printLimitedString(win, 2, 11, "hdg: " + heading_estimator, 14);
        break;
      }


      printLimitedString(win, 4, 11, "ag: " + agl_estimator, 14);

      if (max_flight_z < 0.0) {
        // Negative means EstimationDiagnostics was never received, not that we're near the ceiling.
        printNoData(win, 3, 11, "Max: ", params_.start_minimized);

      } else {
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
  }

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}

void TUI::controlManagerHandler() {
  WINDOW *win = control_manager_window_.get();

  int16_t     color;
  bool        null_tracker, avoiding_collision, bumper_active, can_takeoff;
  double      rate;
  std::string curr_controller, curr_tracker, curr_gains, curr_constraints;
  bool        callbacks_enabled, rc_mode, have_goal, tracking_trajectory;

  {
    std::scoped_lock lock(mutex_status_msg_);
    const auto      &ci = last_control_info_;

    rate = last_system_health_info_.control_manager_rate;

    // "unknown" is ControlInfo's not-yet-received sentinel; .empty() alone never actually catches it.
    curr_controller  = (ci.active_controller.empty() || ci.active_controller == "unknown") ? std::string("NO DATA") : ci.active_controller;
    curr_tracker     = (ci.active_tracker.empty() || ci.active_tracker == "unknown") ? std::string("NO DATA") : ci.active_tracker;
    curr_gains       = (ci.active_gains.empty() || ci.active_gains == "unknown") ? std::string("NO DATA") : ci.active_gains;
    curr_constraints = (ci.active_constraints.empty() || ci.active_constraints == "unknown") ? std::string("NO DATA") : ci.active_constraints;

    callbacks_enabled   = ci.callbacks_enabled;
    rc_mode             = (last_uav_state_.state == mrs_msgs::msg::State::STATE_RC_MODE);
    have_goal           = ci.have_goal;
    tracking_trajectory = ci.tracking_trajectory;
    null_tracker        = (ci.active_tracker == "NullTracker");
    avoiding_collision  = last_collision_avoidance_info_.avoiding_collision;
    bumper_active       = last_collision_avoidance_info_.bumper_active;
    can_takeoff         = last_general_robot_info_.ready_to_start;
  }
  // Nominal MRS control_manager diagnostics rate is 10 Hz.
  color = rateColor(rate, 10.0);

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, bumper_active, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, COLOR_PAIR(color));

  if (params_.start_minimized) {
    printLimitedDouble(win, 0, 1, "Ctr %3.0f", rate, 1000);

    if (rate == 0.0) {

      printNoData(win, 0, 1, params_.start_minimized);
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

      printNoData(win, 0, 1, params_.start_minimized);

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

void TUI::hwApiStateHandler() {
  WINDOW     *win = hw_api_state_window_.get();
  int16_t     color;
  double      hw_api_rate, state_rate, cmd_rate, battery_rate;
  bool        gnss_ok, armed;
  std::string mode;
  double      battery_volt, battery_curr, battery_wh_drained;
  double      thrust, mass_estimate, mass_set, gnss_qual, mag_norm, mag_norm_rate;
  bool        avoiding_collision, bumper_active, can_takeoff, null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    const auto      &bat = last_general_robot_info_.battery_state;

    hw_api_rate = last_system_health_info_.hw_api_rate;
    // The legacy per-topic rates (state/cmd/battery) collapsed into a single
    // hw_api_rate in SystemHealthInfo. Alias them so the existing zero-check UI
    // logic still trips when hw_api stops publishing entirely.
    state_rate   = hw_api_rate;
    cmd_rate     = hw_api_rate;
    battery_rate = hw_api_rate;

    gnss_ok = false;
    if (const auto *gnss = utils::findSensor(last_system_health_info_.available_sensors, mrs_msgs::msg::SensorStatus::TYPE_GNSS); gnss) {
      gnss_ok   = (gnss->level == mrs_msgs::msg::SensorStatus::OK);
      gnss_qual = utils::parseDoubleOr(utils::lookupDetail(gnss->details, "quality"), 0.0);
    }

    mag_norm      = 0.0;
    mag_norm_rate = 0.0;
    if (const auto *mag = utils::findSensor(last_system_health_info_.available_sensors, mrs_msgs::msg::SensorStatus::TYPE_MAGNETOMETER); mag) {
      mag_norm      = utils::parseDoubleOr(utils::lookupDetail(mag->details, "norm_gauss"), 0.0);
      mag_norm_rate = mag->rate;
    }

    armed              = last_uav_info_.armed;
    mode               = last_uav_info_.flight_state;
    battery_volt       = bat.voltage;
    battery_curr       = bat.current;
    battery_wh_drained = bat.wh_drained;
    thrust             = last_control_info_.thrust / 100.0;
    mass_estimate      = last_uav_info_.mass_estimate;
    mass_set           = last_uav_info_.mass_nominal;
    avoiding_collision = last_collision_avoidance_info_.avoiding_collision;
    bumper_active      = last_collision_avoidance_info_.bumper_active;
    can_takeoff        = last_general_robot_info_.ready_to_start;
    null_tracker       = (last_control_info_.active_tracker == "NullTracker");
  }
  // Nominal MRS hw_api rate is 100 Hz.
  color = rateColor(hw_api_rate, 100.0);

  std::string tmp_string;

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, avoiding_collision, bumper_active, can_takeoff, null_tracker);

  if (_light_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  if (params_.start_minimized) {
    printLimitedDouble(win, 0, 1, "Mav %3.0f", hw_api_rate, 1000);
    wattroff(win, COLOR_PAIR(color));

    if (hw_api_rate == 0) {
      printNoData(win, 0, 1, params_.start_minimized);
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

    // battery_rate is aliased to hw_api_rate, so battery_volt < 0 catches BatteryState never arriving.
    if (battery_rate == 0 || battery_volt < 0.0) {

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


    if (cmd_rate == 0 || thrust < 0.0) {

      printLimitedString(win, 3, 5, "ERR", 3);

    } else {

      if (thrust > 0.75) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (thrust > 0.65 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 3, 5, ".%2.0f", thrust * 100, 100);
      wattron(win, COLOR_PAIR(color));
    }

    // mass_nominal/mass_estimate come from UavInfo, independent of the thrust/ControlInfo check above.
    if (mass_set < 0.0 || mass_estimate < 0.0) {

      printNoData(win, 4, 1, params_.start_minimized);

    } else {

      color = static_cast<int>(ColorPair::Green);

      double mass_diff = std::fabs(mass_estimate - mass_set) / mass_set;

      if (mass_diff > 0.3) {

        color = static_cast<int>(ColorPair::Red);

      } else if (mass_diff > 0.2) {

        color = static_cast<int>(ColorPair::Yellow);
      }

      wattron(win, COLOR_PAIR(color));
      printLimitedDouble(win, 4, 1, "%4.1f kg", mass_estimate, 99.99);
    }

    if (!gnss_ok) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 5, "GNSS", 6);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      printLimitedString(win, 1, 5, "GNSS", 6);
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

      printNoData(win, 0, 1, params_.start_minimized);
    }

    if (state_rate == 0) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 1, "State: ", 15);
      printNoData(win, 1, 9, params_.start_minimized);
      printLimitedString(win, 2, 1, "Mode: ", 15);
      printNoData(win, 1, 9, params_.start_minimized);
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

    if (battery_rate == 0 || battery_volt < 0.0) {

      printNoData(win, 4, 1, "Batt:  ", params_.start_minimized);

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
      printLimitedDouble(win, 4, 16, "%6.1fWh", battery_wh_drained, 9999.9);
    }

    if (mag_norm_rate == 0) {

      printNoData(win, 3, 1, "Mag:  ", params_.start_minimized);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (mag_norm > 0.9 || mag_norm < 0.25) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (mag_norm > 0.65) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 3, 1, "Mag: %4.2f", mag_norm, 9.99);
    }

    if (cmd_rate == 0 || thrust < 0.0) {

      printNoData(win, 5, 1, "Thrust: ", params_.start_minimized);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (thrust > 0.75) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (thrust > 0.65 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 5, 1, "Thrust: %4.2f", thrust, 1.01);
      wattron(win, COLOR_PAIR(color));
    }

    if (mass_set < 0.0 || mass_estimate < 0.0) {

      // x=17 clears "Thrust: NO DATA" (cols 1-15) when both blocks are missing at once.
      printNoData(win, 5, 17, params_.start_minimized);

    } else {

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
      printLimitedString(win, 1, 17, "NO_GNSS", 7);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      printLimitedString(win, 1, 17, "GNSS_OK", 7);
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

void TUI::topLineHandler() {
  WINDOW *win = top_bar_window_.get();
  werase(win);

  std::string  uav_name, uav_type;
  bool         collision_avoidance_enabled, avoiding_collision, bumper_active;
  uint16_t     num_other_uavs;
  int          secs_flown;
  bool         have_general_robot_info, have_collision_avoidance_info, have_uav_info;
  rclcpp::Time last_time_got_data;

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_name                      = last_general_robot_info_.robot_name;
    uav_type                      = utils::robotTypeToString(last_general_robot_info_.robot_type);
    collision_avoidance_enabled   = last_collision_avoidance_info_.collision_avoidance_enabled;
    avoiding_collision            = last_collision_avoidance_info_.avoiding_collision;
    bumper_active                 = last_collision_avoidance_info_.bumper_active;
    num_other_uavs                = static_cast<uint16_t>(last_collision_avoidance_info_.other_robots_visible.size());
    secs_flown                    = static_cast<int>(std::max(0.0f, last_uav_info_.flight_duration));
    have_general_robot_info       = last_time_got_general_robot_info_.nanoseconds() != 0;
    have_collision_avoidance_info = last_time_got_collision_avoidance_info_.nanoseconds() != 0;
    have_uav_info                 = last_time_got_uav_info_.nanoseconds() != 0;
    last_time_got_data            = last_time_got_data_;
  }

  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  printLimitedInt(win, 0, 0, "ToF: %i", secs_flown, 1000);

  double since_data_s = (clock_->now() - last_time_got_data).seconds();

  // If we haven't received data for a while, switch to the "no data" color scheme. If we start receiving data again, switch back to the normal color scheme.
  if (const bool nd = (since_data_s < 3.0); nd != have_data_) {
    have_data_ = nd;
    _light_    = setupColors(have_data_, params_.colorscheme, params_.colorblind_mode);
  }

  since_data_s = std::min(since_data_s, 99.9);

  const int status_x = params_.start_minimized ? 27 : 26;
  const int alert_x  = params_.start_minimized ? 22 : 26;
  const int count_x  = params_.start_minimized ? 31 : 51;
  const int uavs_x   = params_.start_minimized ? -1 : 45;

  // x=10 fits the common case ("uav1 - DRONE") without truncation; still capped against status_x.
  if (have_general_robot_info) {
    const std::string name_type = " " + uav_name + " - " + uav_type + " ";
    printLimitedString(win, 0, 10, name_type, static_cast<unsigned long>(std::min(status_x, alert_x) - 11));
  } else {
    printNoData(win, 0, 10, params_.start_minimized);
  }

  const char *disabled_text = params_.start_minimized ? "C/A" : "COL AVOID DISABLED";
  const char *avoiding_text = params_.start_minimized ? "!AVOIDING!" : "!! AVOIDING COLLISION !!";
  const char *bumper_text   = params_.start_minimized ? "!BUMPER!" : "!! BUMPER ACTIVE !!";
  const char *enabled_text  = params_.start_minimized ? "C/A" : "COL AVOID ENABLED,";

  if (!have_collision_avoidance_info) {
    printNoData(win, 0, status_x, params_.start_minimized ? "C/A " : "COL AVOID ", params_.start_minimized);

  } else if (!collision_avoidance_enabled) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    mvwprintw(win, 0, status_x, "%s", disabled_text);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  } else if (bumper_active) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    wattron(win, A_BLINK);
    mvwprintw(win, 0, alert_x, "%s", bumper_text);
    wattroff(win, A_BLINK);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  } else if (avoiding_collision) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    wattron(win, A_BLINK);
    mvwprintw(win, 0, alert_x, "%s", avoiding_text);
    wattroff(win, A_BLINK);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  } else {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
    mvwprintw(win, 0, status_x, "%s", enabled_text);

    if (!params_.start_minimized) {
      mvwprintw(win, 0, uavs_x, "UAVs: ");
    }

    if (num_other_uavs == 0) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }

    printLimitedInt(win, 0, count_x, "%i", num_other_uavs, 100);

    if (num_other_uavs == 0) {
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }

    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
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

  if (have_uav_info) {
    mvwprintw(win, 0, 0, "ToF: %i:%02i", mins, secs);
  } else {
    printNoData(win, 0, 0, "ToF: ", params_.start_minimized);
  }
  wattroff(win, A_BOLD);

  // btop-style hotkey hints on the right of the top bar — the trigger key (the
  // red letter) maps directly to the STANDARD-mode key handler in status.cpp.
  if (!params_.start_minimized && !in_remote_mode_) {
    int hx = 62;
    hx     = printHotkey(win, 0, hx, "menu");
    hx     = printHotkey(win, 0, hx, "goto");
    hx     = printHotkey(win, 0, hx, "Remote");
    hx     = printHotkey(win, 0, hx, "Mini");
    hx     = printHotkey(win, 0, hx, "Display");
    hx     = printHotkey(win, 0, hx, "pane");
    hx     = printHotkey(win, 0, hx, "help");
  }

  wnoutrefresh(win);
}

// | --------------------- Bottom-window helpers --------------- |

void TUI::blankBottomWindow() {
  if ((clock_->now() - bottom_window_clear_time_).seconds() > 3.0) {
    werase(bottom_window_.get());
  }
}

void TUI::renderServiceResult(bool success, const std::string &msg) {
  printServiceResult(bottom_window_.get(), _light_, success, msg);
  bottom_window_clear_time_ = clock_->now();
}

// | --------------------- Menu helpers ----------------------- |

bool TUI::isValidMenuIndex(int index, size_t container_size) {
  return index >= 0 && static_cast<size_t>(index) < container_size;
}

void TUI::clearMenus() {
  menu_vec_.clear();
  submenu_vec_.clear();
}

void TUI::createSubMenu(std::vector<std::string> &submenu_entries) {
  submenu_vec_.clear();
  if (!submenu_entries.empty()) {

    int                  x;
    int                  y;
    [[maybe_unused]] int rows;
    int                  cols;

    getyx(menu_vec_[0].getWin(), x, y);
    getmaxyx(menu_vec_[0].getWin(), rows, cols);

    StatusWindow menu(x, 31 + cols, submenu_entries);
    submenu_vec_.push_back(menu);
  }
}

void TUI::createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<mrs_msgs::srv::String> &service_client) {
  sub_menu_rows_.clear();
  for (const auto &entry : submenu_entries) {
    sub_menu_rows_.push_back({entry, [this, entry, &service_client]() {
                                auto request   = std::make_shared<mrs_msgs::srv::String::Request>();
                                request->value = entry;
                                auto response  = service_client.callSync(request);
                                if (!response) {
                                  renderServiceResult(false, "service could not be called");
                                } else {
                                  renderServiceResult(response.value()->success, response.value()->message);
                                }
                              }});
  }
}

void TUI::createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> &service_client) {
  sub_menu_rows_.clear();
  for (const auto &entry : submenu_entries) {
    if (entry == "CANCEL") {
      sub_menu_rows_.push_back({"CANCEL", []() {}});
      continue;
    }
    sub_menu_rows_.push_back({entry, [this, entry, &service_client]() {
                                auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
                                auto response = service_client.callSync(request);
                                if (!response) {
                                  renderServiceResult(false, "service could not be called");
                                } else {
                                  renderServiceResult(response.value()->success, response.value()->message);
                                }
                              }});
  }
}

void TUI::createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> &service_client) {
  sub_menu_rows_.clear();
  for (const auto &entry : submenu_entries) {
    if (entry == "CANCEL") {
      sub_menu_rows_.push_back({"CANCEL", []() {}});
      continue;
    }
    sub_menu_rows_.push_back({entry, [this, entry, &service_client]() {
                                auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
                                request->data = true;
                                auto response = service_client.callSync(request);
                                if (!response) {
                                  renderServiceResult(false, "service could not be called");
                                } else {
                                  renderServiceResult(response.value()->success, response.value()->message);
                                }
                              }});
  }
}

// | --------------------- Main menu ----------------------- |

void TUI::setupMainMenu() {
  main_menu_rows_.clear();
  main_menu_text_.clear();

  bool null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    null_tracker = (last_control_info_.active_tracker == "NullTracker");
  }

  // Create menu entries for trigger services
  for (auto &service : service_entries_) {
    std::string name = service.display_name;
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    if (null_tracker && (name.find("land") != std::string::npos)) {
      continue;
    }
    if (!null_tracker && (name.find("takeoff") != std::string::npos)) {
      continue;
    }
    main_menu_rows_.push_back({service.display_name, [this, service]() mutable {
                                 std::vector<std::string> menu_text{"CANCEL", service.display_name};
                                 createSubMenu(menu_text);
                                 createSubMenuActions(menu_text, service.client);
                               }});
  }

  // Toggle output service
  main_menu_rows_.push_back({"Toggle Output", [this]() {
                               std::vector<std::string> menu_text{"CANCEL", "Toggle Output"};
                               createSubMenu(menu_text);
                               createSubMenuActions(menu_text, sc_toggle_output_);
                             }});

  // Create menu entries for setting controller, tracker, gains, constraints, estimator
  main_menu_rows_.push_back({"Set Constraints", [this]() {
                               std::vector<std::string> constraints_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 constraints_text = utils::withActiveFirst(last_control_info_.active_constraints, last_control_info_.available_constraints);
                               }
                               sub_menu_rows_.clear();
                               createSubMenu(constraints_text);
                               createSubMenuActions(constraints_text, sc_set_constraints_);
                             }});

  main_menu_rows_.push_back({"Set Gains", [this]() {
                               std::vector<std::string> gains_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 gains_text = utils::withActiveFirst(last_control_info_.active_gains, last_control_info_.available_gains);
                               }
                               createSubMenu(gains_text);
                               createSubMenuActions(gains_text, sc_set_gains_);
                             }});

  main_menu_rows_.push_back({"Set Controller", [this]() {
                               std::vector<std::string> controllers_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 controllers_text = utils::withActiveFirst(last_control_info_.active_controller, last_control_info_.available_controllers);
                               }
                               createSubMenu(controllers_text);
                               createSubMenuActions(controllers_text, sc_set_controller_);
                             }});

  main_menu_rows_.push_back({"Set Tracker", [this]() {
                               std::vector<std::string> trackers_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 trackers_text = utils::withActiveFirst(last_control_info_.active_tracker, last_control_info_.available_trackers);
                               }
                               createSubMenu(trackers_text);
                               createSubMenuActions(trackers_text, sc_set_tracker_);
                             }});

  main_menu_rows_.push_back({"Set Estimator", [this]() {
                               std::vector<std::string> odometry_lat_sources_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 odometry_lat_sources_text =
                                     utils::withActiveFirst(last_state_estimation_info_.current_estimator, last_state_estimation_info_.switchable_estimators);
                               }
                               createSubMenu(odometry_lat_sources_text);
                               createSubMenuActions(odometry_lat_sources_text, sc_set_estimator_);
                             }});

  for (const auto &rows : main_menu_rows_) {
    main_menu_text_.push_back(rows.label);
  }

  StatusWindow menu(1, 32, main_menu_text_);
  menu_vec_.push_back(menu);
}

bool TUI::mainMenuHandler(int key_in) {

  if (!submenu_vec_.empty()) {

    menu_vec_[0].iterate(main_menu_text_, -1, true);

    auto result = submenu_vec_[0].iterate(key_in, true);

    if (result.action == StatusWindow::Result::Action::Exit) {
      submenu_vec_.clear();
      return false;
    }

    if (key_in == static_cast<int>(Key::Enter)) {
      sub_menu_rows_[result.selected_line].on_open();
      submenu_vec_.clear();
      sub_menu_rows_.clear();
      return true;
    }
    return false;
  }

  auto result = menu_vec_[0].iterate(main_menu_text_, key_in, true);

  if (result.action == StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    submenu_vec_.clear();
    return true;
  }

  if (result.pressed_key == static_cast<int>(Key::Enter) && isValidMenuIndex(result.selected_line, main_menu_rows_.size())) {
    main_menu_rows_[result.selected_line].on_open();
  }

  return false;
}

// | --------------------- Goto menu ----------------------- |

void TUI::setupGotoMenu() {
  std::string odom_frame;

  {
    std::scoped_lock lock(mutex_status_msg_);
    odom_frame = last_state_estimation_info_.header.frame_id;
  }

  goto_menu_inputs_.clear();
  goto_menu_text_.clear();
  goto_menu_text_.push_back(" X:                ");
  goto_menu_text_.push_back(" Y:                ");
  goto_menu_text_.push_back(" Z:                ");
  goto_menu_text_.push_back(" hdg:              ");
  goto_menu_text_.push_back(" " + odom_frame + " ");

  StatusWindow menu(1, 32, goto_menu_text_);
  menu_vec_.push_back(menu);

  for (int i = 0; i < 4; i++) {
    ControlBar tmpbox(8, menu.getWin(), goto_double_vec_[i]);
    goto_menu_inputs_.push_back(tmpbox);
  }
}

bool TUI::gotoMenuHandler(int key_in) {

  auto result = menu_vec_[0].iterate(goto_menu_text_, key_in, false);

  if (result.action == StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    return true;
  }

  if (result.pressed_key == static_cast<int>(Key::Enter)) {

    goto_double_vec_[0] = goto_menu_inputs_[0].getDouble();
    goto_double_vec_[1] = goto_menu_inputs_[1].getDouble();
    goto_double_vec_[2] = goto_menu_inputs_[2].getDouble();
    goto_double_vec_[3] = goto_menu_inputs_[3].getDouble();

    auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

    request->reference.position.x = goto_double_vec_[0];
    request->reference.position.y = goto_double_vec_[1];
    request->reference.position.z = goto_double_vec_[2];
    request->reference.heading    = goto_double_vec_[3];

    {
      std::scoped_lock lock(mutex_status_msg_);
      request->header.frame_id = last_state_estimation_info_.header.frame_id;
    }

    auto response = sc_goto_reference_.callSync(request);

    if (!response) {
      renderServiceResult(false, "service could not be called");
      menu_vec_.clear();
      return true;
    }

    renderServiceResult(response.value()->success, response.value()->message);
    menu_vec_.clear();
    return true;

  } else if (isValidMenuIndex(result.selected_line, goto_menu_inputs_.size())) {

    goto_menu_inputs_[result.selected_line].process(result.pressed_key);
  }

  for (size_t i = 0; i < goto_menu_inputs_.size(); i++) {
    if (int(i) == menu_vec_[0].getLine()) {
      goto_menu_inputs_[i].print(i + 1, true);
    } else {
      goto_menu_inputs_[i].print(i + 1, false);
    }
  }

  wnoutrefresh(menu_vec_[0].getWin());
  return false;
}

// | --------------------- Display menu ----------------------- |

void TUI::setupDisplayText() {
  display_menu_text_.clear();

  char                     command[50] = "tmux list-windows | cut -d' ' -f-2";
  std::string              response    = utils::callTerminal(command);
  std::vector<std::string> results     = utils::splitByChar(response, '\n');

  const bool skip_last = !results.empty() && results.back().empty();
  const auto end_index = skip_last ? results.size() - 1 : results.size();
  for (size_t i = 0; i < end_index; i++) {
    display_menu_text_.push_back("[ ] " + results[i]);
  }

  for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
    display_menu_text_[selected_tmux_window_[i]][1] = '*';
  }
}

void TUI::setupDisplayMenu() {
  setupDisplayText();

  StatusWindow menu(1, 32, display_menu_text_);
  menu_vec_.push_back(menu);
}

bool TUI::displayMenuHandler(int key_in) {

  auto result = menu_vec_[0].iterate(display_menu_text_, key_in, false);

  if (result.action == StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    return true;
  }

  if (result.pressed_key == static_cast<int>(Key::Enter)) {

    auto it = std::find(selected_tmux_window_.begin(), selected_tmux_window_.end(), result.selected_line);

    if (it != selected_tmux_window_.end()) {
      display_menu_text_[result.selected_line][1] = ' ';
      selected_tmux_window_.erase(it);
    } else if (int(selected_tmux_window_.size()) < MAX_SELECTED_TMUX_WINDOWS) {
      display_menu_text_[result.selected_line][1] = '*';
      selected_tmux_window_.push_back(result.selected_line);
    }

    std::ofstream outputFile(params_.display_config_filename, std::ofstream::out | std::ofstream::trunc);

    for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
      outputFile << selected_tmux_window_[i] << '\n';
    }
    outputFile.close();
  }

  wnoutrefresh(menu_vec_[0].getWin());
  return false;
}

void TUI::loadDisplayConfig() {
  if (!std::filesystem::exists(params_.display_config_filename)) {
    return;
  }

  selected_tmux_window_.clear();

  std::ifstream file(params_.display_config_filename);
  std::string   line;

  for (int i = 0; i < MAX_SELECTED_TMUX_WINDOWS; i++) {
    std::getline(file, line);
    try {
      selected_tmux_window_.push_back(std::stoi(line));
    }
    catch (const std::invalid_argument &e) {
    }
  }

  file.close();

  setupDisplayText();
}

// | --------------------- Tmux/help rendering --------------- |

// | -------------------------- Remote -------------------------- |

void TUI::enterRemoteMode() {
  remote_hover_ = false;
}

void TUI::remoteHandler(int key) {
  drawRemoteBanner(top_bar_window_.get());

  if (key == 'T') {
    toggleTurboRemote();
    return;
  }

  if (key == 'G') {
    std::scoped_lock lock(mutex_status_msg_);
    if (last_control_info_.flying_normally) {
      remote_global_ = !remote_global_;
    }
    return;
  }

  handleRemoteMotion(key);
}

void TUI::drawRemoteBanner(WINDOW *win) {
  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  const int rem_x   = params_.start_minimized ? 33 : 62;
  const int mode_x  = params_.start_minimized ? 37 : 82;
  const int turbo_x = params_.start_minimized ? 39 : 74;

  const char *rem_text   = params_.start_minimized ? "REM" : "REMOTE MODE";
  const char *mode_text  = remote_global_ ? (params_.start_minimized ? "G" : "GLOBAL MODE") : (params_.start_minimized ? "L" : "LOCAL MODE");
  const char *turbo_text = params_.start_minimized ? "!T!" : "!TURBO!";

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  mvwprintw(win, 0, rem_x, "%s", rem_text);
  mvwprintw(win, 0, mode_x, "%s", mode_text);

  if (turbo_remote_) {
    wattron(win, A_BLINK);
    mvwprintw(win, 0, turbo_x, "%s", turbo_text);
    wattroff(win, A_BLINK);
  }

  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  wattroff(win, A_BOLD);
}

void TUI::handleRemoteMotion(int key) {
  const double xy_step  = turbo_remote_ ? 5.0 : 2.0;
  const double z_step   = turbo_remote_ ? 2.0 : 1.0;
  const double hdg_step = turbo_remote_ ? 1.0 : 0.5;

  auto fly = [&](double vx, double vy, double vz, double vhdg) {
    mrs_msgs::msg::VelocityReference reference{};
    reference.velocity.x       = vx;
    reference.velocity.y       = vy;
    reference.velocity.z       = vz;
    reference.heading_rate     = vhdg;
    reference.use_heading_rate = true;
    remoteModeFly(reference);
    remote_hover_ = true;
  };

  switch (key) {
  case 'w':
  case 'k':
  case KEY_UP:
    fly(xy_step, 0, 0, 0);
    break;
  case 's':
  case 'j':
  case KEY_DOWN:
    fly(-xy_step, 0, 0, 0);
    break;
  case 'a':
  case 'h':
  case KEY_LEFT:
    fly(0, xy_step, 0, 0);
    break;
  case 'd':
  case 'l':
  case KEY_RIGHT:
    fly(0, -xy_step, 0, 0);
    break;

  case 'r':
    fly(0, 0, z_step, 0);
    break;
  case 'f':
    fly(0, 0, -z_step, 0);
    break;

  case 'q':
    fly(0, 0, 0, hdg_step);
    break;
  case 'e':
    fly(0, 0, 0, -hdg_step);
    break;

  default:
    if (remote_hover_) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      sc_hover_.callSync(request);
      remote_hover_ = false;
    }
    break;
  }
}

void TUI::toggleTurboRemote() {
  bool is_flying_normally;
  {
    std::scoped_lock lock(mutex_status_msg_);
    is_flying_normally = last_control_info_.flying_normally;
  }

  if (!is_flying_normally) {
    return;
  }

  if (turbo_remote_) {
    // Toggle down turbo remote after new pressed T
    turbo_remote_  = false;
    auto request   = std::make_shared<mrs_msgs::srv::String::Request>();
    request->value = old_constraints_;
    auto response  = sc_set_constraints_.callSync(request);
    if (!response) {
      renderServiceResult(false, "service could not be called");
      return;
    }
    renderServiceResult(response.value()->success, response.value()->message);
    return;
  }

  // Enable turbo remote constraints
  turbo_remote_ = true;
  {
    std::scoped_lock lock(mutex_status_msg_);
    old_constraints_ = last_control_info_.active_constraints;
  }
  auto request   = std::make_shared<mrs_msgs::srv::String::Request>();
  request->value = params_.turbo_remote_constraints;
  auto response  = sc_set_constraints_.callSync(request);
  if (!response) {
    renderServiceResult(false, "service could not be called");
    return;
  }
  renderServiceResult(response.value()->success, response.value()->message);
}

void TUI::remoteModeFly(const mrs_msgs::msg::VelocityReference &ref_in) {
  auto request = std::make_shared<mrs_msgs::srv::VelocityReferenceStampedSrv::Request>();

  request->reference.reference = ref_in;

  std::string uav_name;

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_name = last_general_robot_info_.robot_name;
  }

  if (remote_global_) {

    request->reference.header.frame_id = uav_name + "/world_origin";

  } else {

    request->reference.header.frame_id = uav_name + "/fcu_untilted";
  }

  request->reference.header.stamp = clock_->now();

  auto response = sc_velocity_reference_.callSync(request);
}

void TUI::renderTmuxOrHelp() {
  if (params_.start_minimized) {
    return;
  }

  WINDOW *debug_window = debug_window_.get();
  WINDOW *sub1         = sub_tmux_window_1_.get();
  WINDOW *sub2         = sub_tmux_window_2_.get();

  // The bottom region (y=13+) is now exclusively the help / tmux-dump overlay —
  // the pane panel moved up into the top-right, so there's no contention.
  if (!selected_tmux_window_.empty()) {
    bool avoiding_collision, bumper_active, can_takeoff, null_tracker;
    {
      std::scoped_lock lock(mutex_status_msg_);
      avoiding_collision = last_collision_avoidance_info_.avoiding_collision;
      bumper_active      = last_collision_avoidance_info_.bumper_active;
      can_takeoff        = last_general_robot_info_.ready_to_start;
      null_tracker       = (last_control_info_.active_tracker == "NullTracker");
    }
    printTmuxDump(debug_window, sub1, sub2, selected_tmux_window_, session_name_, display_menu_text_, MAX_SELECTED_TMUX_WINDOWS, avoiding_collision,
                  bumper_active, can_takeoff, null_tracker);
  } else {
    printHelp(debug_window, help_active_);
  }
}


} // namespace mrs_uav_status::tui
