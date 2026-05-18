#include <mrs_uav_status/status.hpp>

namespace mrs_uav_status
{

using radians = mrs_lib::geometry::radians;

/* Status() //{ */

Status::Status() : Node("mrs_status_menu") {

  initscr();
  start_color();
  cbreak();
  noecho();
  clear();
  nodelay(stdscr, true);
  keypad(stdscr, true);
  timeout(0);
  curs_set(0); // disable cursor
  set_escdelay(0);
  use_default_colors();

  attron(A_BOLD);

  initialize();
}

//}

/* initialize() //{ */

void Status::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // | ---------------------- Param loader ---------------------- |

  prefillUavStatus();

  mrs_lib::ParamLoader param_loader(node_);

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  std::string platform_config_path;

  param_loader.loadParam("platform_config", platform_config_path);

  if (platform_config_path != "") {
    param_loader.addYamlFile(platform_config_path);
  }

  param_loader.addYamlFileFromParam("config_public");

  param_loader.loadParam("pwd", _pwd_);

  param_loader.loadParam("colorscheme", _colorscheme_);

  std::string turbo_remote_constraints;
  param_loader.loadParam("mrs_uav_status/turbo_remote_constraints", turbo_remote_constraints);

  param_loader.loadParam("mrs_uav_status/colorblind_mode", _colorblind_mode_);
  param_loader.loadParam("mrs_uav_status/enable_profiler", _profiler_enabled_);

  param_loader.loadParam("mrs_uav_status/start_minimized", mini_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  } else {
    RCLCPP_INFO(node_->get_logger(), "All params loaded!");
  }

  // | ------------------------- Timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&Status::timerStatusFast, this);

    timer_status_fast_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(20.0, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&Status::timerStatusSlow, this);

    timer_status_slow_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(1.0, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&Status::timerResize, this);

    timer_resize_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(1.0, clock_), callback_fcn);
  }

  // | ------------------------ Subscribers ------------------------ |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_uav_status_       = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>(shopts, "~/uav_status_in", &Status::callbackUavStatus, this);
  sh_uav_status_short_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatusShort>(shopts, "~/uav_status_short_in", &Status::callbackUavStatusShort, this);

  // mrs_lib profiler
  profiler_ = mrs_lib::Profiler(node_, "Status", _profiler_enabled_);

  // --------------------------------------------------------------
  // |            Window creation and topic association           |
  // --------------------------------------------------------------

  updateTermSize();
  setupWindows();

  _display_config_filename_ = _pwd_ + "/.mrs_status_display_config~";

  tui_ = std::make_unique<tui::TUI>(node_, cbkgrp_sc_, _colorscheme_, _colorblind_mode_, mini_, _display_config_filename_, turbo_remote_constraints);
  {
    std::scoped_lock lock(mutex_status_msg_);
    tui_->onUavStatus(uav_status_);
  }
  tui_->bindBottomWindow(bottom_window_);
  tui_->loadDisplayConfig();

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* setupWindows() //{ */

void Status::setupWindows() {

  if (mini_) {

    control_manager_window_ = newwin(4, 9, 1, 1);
    uav_state_window_       = newwin(6, 9, 5, 1);
    top_bar_window_         = newwin(1, 140, 0, 1);
    general_info_window_    = newwin(4, 9, 1, 10);
    hw_api_state_window_    = newwin(6, 9, 5, 10);
    debug_window_           = newwin(lines_ - 15, cols_ - 1, 13, 1);
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
    debug_window_           = newwin(lines_ - 15, cols_ - 1, 13, 1);
    int half_lines          = (lines_ - 18) / 2;
    sub_tmux_window_1_      = derwin(debug_window_, half_lines, cols_ - 3, 1, 1);
    sub_tmux_window_2_      = derwin(debug_window_, half_lines, cols_ - 3, half_lines + 2, 1);

    generic_topic_window_ = newwin(11, 25, 1, 52);
    string_window_        = newwin(11, 32, 1, 77);
    node_stats_window_    = newwin(11, 50, 1, 109);
  }

  clear();
  tui::setupColors(false, _colorscheme_, _colorblind_mode_);
}

//}

/* timerResize() //{ */

void Status::timerResize() {

  if (!initialized_) {
    return;
  }

  if (updateTermSize()) {

    if (cols_ > 30) {
      resize_term(lines_, cols_);
      setupWindows();
    }
  }
}

//}

/* updateTermSize() //{ */

bool Status::updateTermSize() {

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

  if (cols_ != cols || lines_ != lines) {
    lines_  = lines;
    cols_   = cols;
    changed = true;
  }

  return (changed);
}


//}

/* timerStatusFast() //{ */

void Status::timerStatusFast() {

  if (!initialized_) {
    return;
  }

  tui_->topLineHandler(top_bar_window_, mini_);

  tui_->renderTmuxOrHelp(debug_window_, sub_tmux_window_1_, sub_tmux_window_2_, mini_, help_active_);

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("uavStateHandler");
    tui_->uavStateHandler(uav_state_window_, mini_);
  }

  tui_->maybeBlankBottomWindow();

  int  key_in = getch();
  bool is_flying_normally_;

  switch (state_) {

    /* STANDARD //{ */

  case StatusState::STANDARD: {

    switch (key_in) {

      /* R //{ */

    case 'R': {

      {
        std::scoped_lock lock(mutex_status_msg_);
        is_flying_normally_ = uav_status_.flying_normally;
      }

      if (is_flying_normally_) {
        tui_->enterRemoteMode();
        state_ = StatusState::REMOTE;
      }

      break;
    }

      //}

      /* G //{ */

    case 'G': {

      tui_->resetGimbalCommand();
      state_ = StatusState::GIMBAL;
      break;

    case 'm':
      tui_->setupMainMenu();
      state_ = StatusState::MAIN_MENU;
      break;

    case 'g':
      tui_->setupGotoMenu();
      state_ = StatusState::GOTO_MENU;
      break;

    case 'h':
      help_active_ = !help_active_;
      break;

    case 'M':
      mini_ = !mini_;
      setupWindows();
      timerStatusFast();
      timerStatusSlow();
      break;

    case 'D':
      tui_->setupDisplayMenu();
      state_ = StatusState::DISPLAY_MENU;
      break;

    default:
      flushinp();
      break;
    }
    }

    //}

    break;
  }

    //}

    /* REMOTE //{ */

  case StatusState::REMOTE: {

    flushinp();
    tui_->remoteHandler(key_in, top_bar_window_, mini_);

    if (key_in == 'R' || key_in == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {
      state_ = StatusState::STANDARD;
    }

    break;
  }

    //}

    /* GIMBAL //{ */

  case StatusState::GIMBAL: {

    flushinp();

    tui_->gimbalHandler(key_in, top_bar_window_, mini_);

    if (key_in == 'G' || key_in == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {
      state_ = StatusState::STANDARD;
    }

    break;
  }

    //}

    /* MAIN_MENU //{ */

  case StatusState::MAIN_MENU: {

    flushinp();

    if (tui_->mainMenuHandler(key_in)) {

      tui_->clearMenus();

      wnoutrefresh(debug_window_);
      wnoutrefresh(bottom_window_);

      state_ = StatusState::STANDARD;
    }

    break;
  }

    //}

    /* GOTO_MENU //{ */

  case StatusState::GOTO_MENU: {

    flushinp();

    if (tui_->gotoMenuHandler(key_in)) {
      tui_->clearMenus();
      state_ = StatusState::STANDARD;
    }

    break;
  }

    //}

    /* DISPLAY_MENU //{ */

  case StatusState::DISPLAY_MENU: {

    flushinp();

    if (tui_->displayMenuHandler(key_in)) {
      tui_->clearMenus();
      state_ = StatusState::STANDARD;
    }

    break;
  }

    //}
  }

  if (state_ != StatusState::MAIN_MENU && state_ != StatusState::GOTO_MENU && state_ != StatusState::DISPLAY_MENU) {
    wnoutrefresh(bottom_window_);
  }

  wnoutrefresh(top_bar_window_);
  doupdate();
}

//}

/* timerStatusSlow() //{ */

void Status::timerStatusSlow() {

  if (!initialized_) {
    return;
  }

  tui_->tickSlowCounter();

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("hwApiStateHandler");
    tui_->hwApiStateHandler(hw_api_state_window_, mini_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("controlManagerHandler");
    tui_->controlManagerHandler(control_manager_window_, mini_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("genericTopicHandler");
    tui_->genericTopicHandler(generic_topic_window_, mini_);
  }

  if (!mini_) {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("nodeStatsHandler");
    tui_->nodeStatsHandler(node_stats_window_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("stringHandler");
    tui_->stringHandler(string_window_, mini_);
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("generalInfoHandler");
    tui_->generalInfoHandler(general_info_window_, mini_);
  }
}

//}

/* HANDLERS //{ */



/* callbackUavStatus() //{ */

void Status::callbackUavStatus(const mrs_msgs::msg::UavStatus::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_status_ = *msg;
  }

  tui_->onUavStatus(*msg);
}

//}

/* callbackUavStatusShort() //{ */

void Status::callbackUavStatusShort(const mrs_msgs::msg::UavStatusShort::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_status_msg_);

    uav_status_.odom_x     = msg->odom_x;
    uav_status_.odom_y     = msg->odom_y;
    uav_status_.odom_z     = msg->odom_z;
    uav_status_.odom_hdg   = msg->odom_hdg;
    uav_status_.odom_color = msg->odom_color;
    uav_status_.odom_hz    = msg->odom_hz;

    uav_status_.cmd_x   = msg->cmd_x;
    uav_status_.cmd_y   = msg->cmd_y;
    uav_status_.cmd_z   = msg->cmd_z;
    uav_status_.cmd_hdg = msg->cmd_hdg;
  }

  tui_->onUavStatusShort(*msg);
}

//}

/* prefillUavStatus() //{ */

void Status::prefillUavStatus() {
  std::scoped_lock lock(mutex_status_msg_);

  uav_status_.uav_name                = "N/A";
  uav_status_.uav_type                = "N/A";
  uav_status_.uav_mass                = "N/A";
  uav_status_.control_manager_diag_hz = 0.0;
  uav_status_.controllers.clear();
  uav_status_.gains.clear();
  uav_status_.trackers.clear();
  uav_status_.constraints.clear();
  uav_status_.secs_flown = 0;
  uav_status_.odom_hz    = 0.0;
  uav_status_.odom_x     = 0.0;
  uav_status_.odom_y     = 0.0;
  uav_status_.odom_z     = 0.0;
  uav_status_.odom_hdg   = 0.0;
  uav_status_.odom_frame = "N/A";
  uav_status_.odom_estimators.clear();
  uav_status_.max_flight_z          = 0.0;
  uav_status_.cpu_load              = 0.0;
  uav_status_.cpu_ghz               = 0.0;
  uav_status_.free_ram              = 0.0;
  uav_status_.free_hdd              = 0.0;
  uav_status_.hw_api_hz             = 0.0;
  uav_status_.hw_api_armed          = false;
  uav_status_.hw_api_mode           = "N/A";
  uav_status_.hw_api_gnss_ok        = false;
  uav_status_.hw_api_gnss_qual      = 0.0;
  uav_status_.hw_api_gnss_fix_type  = 0;
  uav_status_.hw_api_gnss_num_sats  = 0;
  uav_status_.hw_api_gnss_pos_acc   = 0.0;
  uav_status_.hw_api_gnss_status_hz = 0.0;
  uav_status_.battery_volt          = 0.0;
  uav_status_.battery_curr          = 0.0;
  uav_status_.thrust                = 0.0;
  uav_status_.mass_estimate         = 0.0;
  uav_status_.mass_set              = 0.0;
  uav_status_.custom_topics.clear();
  uav_status_.custom_string_outputs.clear();
  uav_status_.flying_normally     = false;
  uav_status_.null_tracker        = true;
  uav_status_.have_goal           = false;
  uav_status_.rc_mode             = false;
  uav_status_.tracking_trajectory = false;
  uav_status_.callbacks_enabled   = false;
}

//}


} // namespace mrs_uav_status

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mrs_uav_status::Status>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
