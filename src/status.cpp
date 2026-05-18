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

  param_loader.loadParam("mrs_uav_status/turbo_remote_constraints", _turbo_remote_constraints_);

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

  // | ------------------------ Publishers ------------------------ |

  ph_gimbal_state_ = mrs_lib::PublisherHandler<mrs_msgs::msg::GimbalState>(node_, "~/gimbal_command_out");

  // | --------------------- service clients -------------------- |

  sc_goto_reference_  = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/reference_out", cbkgrp_sc_);
  sc_set_constraints_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_constraints_out", cbkgrp_sc_);
  sc_hover_           = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/hover_out", cbkgrp_sc_);

  // mrs_lib profiler
  profiler_ = mrs_lib::Profiler(node_, "Status", _profiler_enabled_);

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);

  // --------------------------------------------------------------
  // |            Window creation and topic association           |
  // --------------------------------------------------------------

  updateTermSize();
  setupWindows();

  _display_config_filename_ = _pwd_ + "/.mrs_status_display_config~";

  tui_ = std::make_unique<tui::TUI>(node_, cbkgrp_sc_, _colorscheme_, _colorblind_mode_, mini_, _display_config_filename_, _turbo_remote_constraints_);
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
  _light_ = tui::setupColors(false, _colorscheme_, _colorblind_mode_);
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
        remote_hover_ = false;
        state_        = StatusState::REMOTE;
      }

      break;
    }

      //}

      /* G //{ */

    case 'G': {

      gimbal_command_.fpv_mode    = true;
      gimbal_command_.is_on       = true;
      gimbal_command_.gimbal_pan  = 1500;
      gimbal_command_.gimbal_tilt = 1500;
      state_                      = StatusState::GIMBAL;
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
    remoteHandler(key_in, top_bar_window_);

    if (key_in == 'R' || key_in == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {

      if (turbo_remote_) {

        turbo_remote_ = false;

        auto request = std::make_shared<mrs_msgs::srv::String::Request>();

        request->value = old_constraints_;

        auto response = sc_set_constraints_.callSync(request);

        if (response) {
          tui_->renderServiceResult(response.value()->success, response.value()->message);
        } else {
          tui_->renderServiceResult(false, "service could not be called");
        }
      }

      state_ = StatusState::STANDARD;
    }

    break;
  }

    //}

    /* GIMBAL //{ */

  case StatusState::GIMBAL: {

    flushinp();

    gimbalHandler(key_in, top_bar_window_);

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


/* remoteHandler() //{ */

void Status::remoteHandler(int key, WINDOW *win) {
  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(mrs_uav_status::tui::ColorPair::Red)));
  if (mini_) {
    mvwprintw(win, 0, 33, "REM");
  } else {
    mvwprintw(win, 0, 55, "REMOTE MODE");
  }

  if (remote_global_) {
    if (mini_) {
      mvwprintw(win, 0, 37, "G");
    } else {
      mvwprintw(win, 0, 75, "GLOBAL MODE");
    }
  } else {
    if (mini_) {
      mvwprintw(win, 0, 37, "L");
    } else {
      mvwprintw(win, 0, 75, "LOCAL MODE");
    }
  }

  if (turbo_remote_) {
    wattron(win, A_BLINK);
    if (mini_) {
      mvwprintw(win, 0, 39, "!T!");
    } else {
      mvwprintw(win, 0, 67, "!TURBO!");
    }
    wattroff(win, A_BLINK);
  }

  wattroff(win, COLOR_PAIR(static_cast<int>(mrs_uav_status::tui::ColorPair::Red)));

  mrs_msgs::msg::Reference       reference;
  mrs_msgs::srv::String::Request string_service;

  reference.position.x = 0.0;
  reference.position.y = 0.0;
  reference.position.z = 0.0;
  reference.heading    = 0.0;

  switch (key) {

  case 'w':
  case 'k':
  case KEY_UP:
    reference.position.x = 2.0;

    if (turbo_remote_) {
      reference.position.x = 5.0;
    }
    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 's':
  case 'j':
  case KEY_DOWN:
    reference.position.x = -2.0;

    if (turbo_remote_) {
      reference.position.x = -5.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'a':
  case 'h':
  case KEY_LEFT:
    reference.position.y = 2.0;

    if (turbo_remote_) {
      reference.position.y = 5.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'd':
  case 'l':
  case KEY_RIGHT:
    reference.position.y = -2.0;

    if (turbo_remote_) {
      reference.position.y = -5.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'r':
    reference.position.z = 1.0;

    if (turbo_remote_) {
      reference.position.z = 2.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'f':
    reference.position.z = -1.0;

    if (turbo_remote_) {
      reference.position.z = -2.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'q':
    reference.heading = 0.5;

    if (turbo_remote_) {
      reference.heading = 1.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'e':
    reference.heading = -0.5;

    if (turbo_remote_) {
      reference.heading = -1.0;
    }

    remoteModeFly(reference);
    remote_hover_ = true;
    break;

  case 'T':

    bool is_flying_normally_;

    {
      std::scoped_lock lock(mutex_status_msg_);
      is_flying_normally_ = uav_status_.flying_normally;
    }

    if (is_flying_normally_) {

      if (turbo_remote_) {

        turbo_remote_ = false;

        auto request = std::make_shared<mrs_msgs::srv::String::Request>();

        request->value = old_constraints_;

        auto response = sc_set_constraints_.callSync(request);

        if (response) {
          tui_->renderServiceResult(response.value()->success, response.value()->message);
        } else {
          tui_->renderServiceResult(false, "service could not be called");
        }

      } else {

        turbo_remote_ = true;

        {
          std::scoped_lock lock(mutex_status_msg_);
          old_constraints_ = uav_status_.constraints[0];
        }

        auto request = std::make_shared<mrs_msgs::srv::String::Request>();

        request->value = _turbo_remote_constraints_;

        auto response = sc_set_constraints_.callSync(request);

        if (response) {
          tui_->renderServiceResult(response.value()->success, response.value()->message);
        } else {
          tui_->renderServiceResult(false, "service could not be called");
        }
      }
    }

    break;

  case 'G': {

    {
      std::scoped_lock lock(mutex_status_msg_);
      is_flying_normally_ = uav_status_.flying_normally;
    }

    if (is_flying_normally_) {
      remote_global_ = !remote_global_;
    }

    break;
  }

  default: {

    if (remote_hover_) {

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

      sc_hover_.callSync(request);

      remote_hover_ = false;
    }

    break;
  }
  }

  wattroff(win, A_BOLD);
}

//}

/* gimbalHandler() //{ */

void Status::gimbalHandler(int key, WINDOW *win) {
  if (_light_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(mrs_uav_status::tui::ColorPair::Red)));
  mvwprintw(win, 0, 43, "GIMBAL      MODE IS ACTIVE");

  if (gimbal_command_.fpv_mode) {
    mvwprintw(win, 0, 50, "FPV");
  } else {
    mvwprintw(win, 0, 50, "P-T");
  }

  wattroff(win, COLOR_PAIR(static_cast<int>(mrs_uav_status::tui::ColorPair::Red)));

  const uint16_t gimbal_max       = 2000;
  const uint16_t gimbal_min       = 1000;
  uint16_t       gimbal_increment = 10;

  switch (key) {

  case 'w':
  case 'k':
  case KEY_UP:
    gimbal_command_.gimbal_tilt -= gimbal_increment;
    break;

  case 's':
  case 'j':
  case KEY_DOWN:
    gimbal_command_.gimbal_tilt += gimbal_increment;
    break;

  case 'a':
  case 'h':
  case KEY_LEFT:
    gimbal_command_.gimbal_pan -= gimbal_increment;
    break;

  case 'd':
  case 'l':
  case KEY_RIGHT:
    gimbal_command_.gimbal_pan += gimbal_increment;
    break;

  case 'm':
    gimbal_command_.fpv_mode = !gimbal_command_.fpv_mode;
    break;

  case 'o':
    gimbal_command_.is_on = !gimbal_command_.is_on;
    break;

  case 'r':
    gimbal_command_.is_on       = true;
    gimbal_command_.fpv_mode    = true;
    gimbal_command_.gimbal_tilt = 1500;
    gimbal_command_.gimbal_pan  = 1500;
    break;
  }

  if (gimbal_command_.gimbal_pan > gimbal_max) {
    gimbal_command_.gimbal_pan = gimbal_max;
  }
  if (gimbal_command_.gimbal_tilt > gimbal_max) {
    gimbal_command_.gimbal_tilt = gimbal_max;
  }

  if (gimbal_command_.gimbal_pan < gimbal_min) {
    gimbal_command_.gimbal_pan = gimbal_min;
  }
  if (gimbal_command_.gimbal_tilt < gimbal_min) {
    gimbal_command_.gimbal_tilt = gimbal_min;
  }

  ph_gimbal_state_.publish(gimbal_command_);

  wattroff(win, A_BOLD);
}

//}

/* remoteModeFly() //{ */

void Status::remoteModeFly(const mrs_msgs::msg::Reference &ref_in) {
  auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

  if (remote_global_) {

    double      cmd_x;
    double      cmd_y;
    double      cmd_z;
    double      cmd_hdg;
    std::string odom_frame;

    {
      std::scoped_lock lock(mutex_status_msg_);
      cmd_x      = uav_status_.cmd_x;
      cmd_y      = uav_status_.cmd_y;
      cmd_z      = uav_status_.cmd_z;
      cmd_hdg    = uav_status_.cmd_hdg;
      odom_frame = uav_status_.odom_frame;
    }

    request->reference.position.x = cmd_x + ref_in.position.x;
    request->reference.position.y = cmd_y + ref_in.position.y;
    request->reference.position.z = cmd_z + ref_in.position.z;
    request->reference.heading    = cmd_hdg + ref_in.heading;
    request->header.frame_id      = odom_frame;

  } else {

    request->reference = ref_in;

    std::string uav_name;
    double      cmd_x;
    double      cmd_y;
    double      cmd_z;
    double      cmd_hdg;
    std::string odom_frame;

    {
      std::scoped_lock lock(mutex_status_msg_);
      uav_name   = uav_status_.uav_name;
      cmd_x      = uav_status_.cmd_x;
      cmd_y      = uav_status_.cmd_y;
      cmd_z      = uav_status_.cmd_z;
      cmd_hdg    = uav_status_.cmd_hdg;
      odom_frame = uav_status_.odom_frame;
    }

    mrs_msgs::msg::ReferenceStamped cmd_reference;

    cmd_reference.reference.position.x = cmd_x;
    cmd_reference.reference.position.y = cmd_y;
    cmd_reference.reference.position.z = cmd_z;
    cmd_reference.reference.heading    = cmd_hdg;
    cmd_reference.header.frame_id      = odom_frame;

    request->header.frame_id = uav_name + "/fcu_untilted";
    request->header.stamp    = clock_->now();

    auto response = transformer_->transformSingle(cmd_reference, request->header.frame_id);
    if (response) {
      cmd_reference = response.value();
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Transform failed when transforming cmd_reference.");
      return;
    }

    request->reference = cmd_reference.reference;
    request->reference.position.x += ref_in.position.x;
    request->reference.position.y += ref_in.position.y;
    request->reference.position.z += ref_in.position.z;
    request->reference.heading += ref_in.heading;
    request->header.frame_id = cmd_reference.header.frame_id;
  }

  request->header.stamp = clock_->now();

  auto response = sc_goto_reference_.callSync(request);
}

//}


//}

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
