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

  bottom_window_clear_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

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
  sc_set_constraints_      = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_constraints_out", cbkgrp_sc_);
  sc_set_gains_            = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_gains_out", cbkgrp_sc_);
  sc_set_controller_       = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_controller_out", cbkgrp_sc_);
  sc_set_tracker_          = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_tracker_out", cbkgrp_sc_);
  sc_set_estimator_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "~/set_estimator_out", cbkgrp_sc_);
  sc_hover_                = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/hover_out", cbkgrp_sc_);

  // mrs_lib profiler
  profiler_ = mrs_lib::Profiler(node_, "Status", _profiler_enabled_);

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);

  // Loads the default GoTo value
  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(2.0);
  goto_double_vec_.push_back(1.57);

  // Loads the default menu options
  service_input_vec_.push_back("uav_manager/land Land");
  service_input_vec_.push_back("uav_manager/land_home Land Home");
  service_input_vec_.push_back("uav_manager/takeoff Takeoff");

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

  if (std::filesystem::exists(_display_config_filename_)) {

    selected_tmux_window_.clear();

    std::ifstream file(_display_config_filename_);

    std::string line;

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

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* setupWindows() //{ */

void Status::setupWindows() {

  std::string command = "tmux display-message -p '#S'";
  session_name_       = utils::callTerminal(command.c_str());
  session_name_.erase(std::remove(session_name_.begin(), session_name_.end(), '\n'), session_name_.end());

  command                           = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string              response = utils::callTerminal(command.c_str());
  std::vector<std::string> results;
  results = mrs_uav_status::utils::splitByChar(response, 'x');

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

  if (!mini_) {
    if (!selected_tmux_window_.empty()) {
      bool avoiding_collision, can_takeoff, null_tracker;
      {
        std::scoped_lock lock(mutex_status_msg_);
        avoiding_collision = uav_status_.avoiding_collision;
        can_takeoff        = uav_status_.automatic_start_can_takeoff;
        null_tracker       = uav_status_.null_tracker;
      }
      tui::printTmuxDump(debug_window_, sub_tmux_window_1_, sub_tmux_window_2_, selected_tmux_window_, session_name_, display_menu_text_,
                         MAX_SELECTED_TMUX_WINDOWS, avoiding_collision, can_takeoff, null_tracker);
    } else {
      tui::printHelp(debug_window_, help_active_);
    }
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("uavStateHandler");
    tui_->uavStateHandler(uav_state_window_, mini_);
  }

  if ((clock_->now() - bottom_window_clear_time_).seconds() > 3.0) {
    werase(bottom_window_);
  }

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
      setupMainMenu();
      state_ = StatusState::MAIN_MENU;
      break;

    case 'g':
      setupGotoMenu();
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
      setupDisplayMenu();
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
          tui::printServiceResult(bottom_window_, _light_, response.value()->success, response.value()->message);
        } else {
          tui::printServiceResult(bottom_window_, _light_, false, "service could not be called");
        }
        bottom_window_clear_time_ = clock_->now();
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

    if (mainMenuHandler(key_in)) {

      menu_vec_.clear();
      submenu_vec_.clear();

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

    if (gotoMenuHandler(key_in)) {
      menu_vec_.clear();
      submenu_vec_.clear();
      state_ = StatusState::STANDARD;
    }

    break;
  }

    //}

    /* DISPLAY_MENU //{ */

  case StatusState::DISPLAY_MENU: {

    flushinp();

    if (displayMenuHandler(key_in)) {
      menu_vec_.clear();
      submenu_vec_.clear();
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

/* isValidMenuIndex() //{ */

bool Status::isValidMenuIndex(int index, size_t container_size) {
  return index >= 0 && static_cast<size_t>(index) < container_size;
}

//}

/* mainMenuHandler() //{ */

bool Status::mainMenuHandler(int key_in) {

  /* SUBMENU IS OPEN //{ */

  if (!submenu_vec_.empty()) {
    // SUBMENU IS OPEN

    menu_vec_[0].iterate(main_menu_text_, -1, true);

    auto result = submenu_vec_[0].iterate(key_in, true);

    if (result.action == mrs_uav_status::tui::StatusWindow::Result::Action::Exit) {
      submenu_vec_.clear();
      return false;
    }

    if (key_in == static_cast<int>(mrs_uav_status::tui::Key::Enter)) {
      sub_menu_rows_[result.selected_line].on_open();
      submenu_vec_.clear();
      sub_menu_rows_.clear();
      return true;
    }
    return false;
    //}

    /* MAIN MENU CASE //{ */
  }
  // MAIN MENU CASE - NO SUBMENU

  auto result = menu_vec_[0].iterate(main_menu_text_, key_in, true);

  if (result.action == mrs_uav_status::tui::StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    submenu_vec_.clear();
    return true;
  }

  if (result.pressed_key == static_cast<int>(mrs_uav_status::tui::Key::Enter) && isValidMenuIndex(result.selected_line, main_menu_rows_.size())) {
    main_menu_rows_[result.selected_line].on_open();
  }

  return false;
  //}
}

//}

/* gotoMenuHandler() //{ */

bool Status::gotoMenuHandler(int key_in) {

  // optional<tuple<int, int>> ret = menu_vec_[0].iterate(goto_menu_text_, key_in, false);
  auto result = menu_vec_[0].iterate(goto_menu_text_, key_in, false);

  if (result.action == mrs_uav_status::tui::StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    return true;
  }

  if (result.pressed_key == static_cast<int>(mrs_uav_status::tui::Key::Enter)) {

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
      request->header.frame_id = uav_status_.odom_frame;
    }

    auto response = sc_goto_reference_.callSync(request);

    if (response) {
      tui::printServiceResult(bottom_window_, _light_, response.value()->success, response.value()->message);
    } else {
      tui::printServiceResult(bottom_window_, _light_, false, "service could not be called");
    }
    bottom_window_clear_time_ = clock_->now();

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

//}

/* displayMenuHandler() //{ */

bool Status::displayMenuHandler(int key_in) {

  // optional<tuple<int, int>> ret = menu_vec_[0].iterate(display_menu_text_, key_in, false);
  auto result = menu_vec_[0].iterate(display_menu_text_, key_in, false);

  if (result.action == mrs_uav_status::tui::StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    return true;
  }

  if (result.pressed_key == static_cast<int>(mrs_uav_status::tui::Key::Enter)) {

    auto it = std::find(selected_tmux_window_.begin(), selected_tmux_window_.end(), result.selected_line);

    if (it != selected_tmux_window_.end()) {
      display_menu_text_[result.selected_line][1] = ' ';
      selected_tmux_window_.erase(it);
    } else if (int(selected_tmux_window_.size()) < MAX_SELECTED_TMUX_WINDOWS) {
      display_menu_text_[result.selected_line][1] = '*';
      selected_tmux_window_.push_back(result.selected_line);
    }

    std::ofstream outputFile(_display_config_filename_, std::ofstream::out | std::ofstream::trunc);

    for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
      outputFile << selected_tmux_window_[i] << '\n';
    }
    outputFile.close();
  }

  wnoutrefresh(menu_vec_[0].getWin());
  return false;
}

//}

void Status::createSubMenu(std::vector<std::string> &submenu_entries) {
  submenu_vec_.clear();
  if (!submenu_entries.empty()) {

    int                  x;
    int                  y;
    [[maybe_unused]] int rows;
    int                  cols;

    getyx(menu_vec_[0].getWin(), x, y);
    getmaxyx(menu_vec_[0].getWin(), rows, cols);

    mrs_uav_status::tui::StatusWindow menu(x, 31 + cols, submenu_entries);
    submenu_vec_.push_back(menu);
  }
}

void Status::createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<mrs_msgs::srv::String> &service_client) {
  sub_menu_rows_.clear();
  for (const auto &entry : submenu_entries) {
    sub_menu_rows_.push_back({entry, [this, entry, &service_client]() {
                                auto request   = std::make_shared<mrs_msgs::srv::String::Request>();
                                request->value = entry;
                                auto response  = service_client.callSync(request);
                                if (!response) {
                                  tui::printServiceResult(bottom_window_, _light_, false, "service could not be called");
                                } else {
                                  tui::printServiceResult(bottom_window_, _light_, response.value()->success, response.value()->message);
                                }
                                bottom_window_clear_time_ = clock_->now();
                              }});
  }
}

void Status::createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> &service_client) {
  sub_menu_rows_.clear();
  for (const auto &entry : submenu_entries) {
    if (entry == "CANCEL") {
      // Empty action for cancel, service call for the actual action
      sub_menu_rows_.push_back({"CANCEL", []() {}});
      continue;
    }
    // Add service call action for other entries
    sub_menu_rows_.push_back({entry, [this, entry, &service_client]() {
                                auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
                                auto response = service_client.callSync(request);
                                if (!response) {
                                  tui::printServiceResult(bottom_window_, _light_, false, "service could not be called");
                                } else {
                                  tui::printServiceResult(bottom_window_, _light_, response.value()->success, response.value()->message);
                                }
                                bottom_window_clear_time_ = clock_->now();
                              }});
  }
}

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
          tui::printServiceResult(bottom_window_, _light_, response.value()->success, response.value()->message);
        } else {
          tui::printServiceResult(bottom_window_, _light_, false, "service could not be called");
        }
        bottom_window_clear_time_ = clock_->now();

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
          tui::printServiceResult(bottom_window_, _light_, response.value()->success, response.value()->message);
        } else {
          tui::printServiceResult(bottom_window_, _light_, false, "service could not be called");
        }
        bottom_window_clear_time_ = clock_->now();
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

/* MENU SETUP //{ */

/* setupMainMenu() //{ */

void Status::setupMainMenu() {
  service_vec_.clear();

  bool null_tracker;

  {
    std::scoped_lock lock(mutex_status_msg_);
    null_tracker = uav_status_.null_tracker;
  }

  for (unsigned long i = 0; i < service_input_vec_.size(); i++) {

    // TODO, fix this with proper flying state instead of null tracker
    if (null_tracker && (i == 0 || i == 1)) {
      continue; // disable land and land home if we are not flying
    }

    if (!null_tracker && i == 2) {

      continue; // disable takeoff if flying
    }

    std::vector<std::string> results;
    results = mrs_uav_status::utils::splitByChar(service_input_vec_[i], ' '); // split the input string into words and put them in results vector

    for (unsigned long j = 2; j < results.size(); j++) {
      results[1] = results[1] + " " + results[j];
    }

    std::string service_name;

    if (results[0].at(0) == '/') {
      service_name = results[0];

    } else {


      std::string uav_name;

      {
        std::scoped_lock lock(mutex_status_msg_);
        uav_name = uav_status_.uav_name;
      }

      service_name = "/" + uav_name + "/" + results[0];
    }

    Service tmp_service(service_name, results[1]);

    tmp_service.service_client = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, service_name);

    service_vec_.push_back(tmp_service);
  }

  main_menu_rows_.clear();
  main_menu_text_.clear();

  for (auto &service : service_vec_) {
    main_menu_rows_.push_back({service.service_display_name, [this, &service]() {
                                 std::vector<std::string> menu_text{"CANCEL", service.service_display_name};
                                 // Create the submenu with the service action
                                 createSubMenu(menu_text);
                                 // Add action for the service
                                 createSubMenuActions(menu_text, service.service_client);
                               }});
  }

  main_menu_rows_.push_back({"Set Constraints", [this]() {
                               std::vector<std::string> constraints_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 constraints_text = uav_status_.constraints;
                               }
                               sub_menu_rows_.clear();
                               // Create the submenu with the constraints
                               createSubMenu(constraints_text);
                               // Add actions for each constraint
                               createSubMenuActions(constraints_text, sc_set_constraints_);
                             }});

  main_menu_rows_.push_back({"Set Gains", [this]() {
                               std::vector<std::string> gains_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 gains_text = uav_status_.gains;
                               }
                               // Create the submenu with the gains
                               createSubMenu(gains_text);
                               // Add actions for each gain
                               createSubMenuActions(gains_text, sc_set_gains_);
                             }});

  main_menu_rows_.push_back({"Set Controller", [this]() {
                               std::vector<std::string> controllers_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 controllers_text = uav_status_.controllers;
                               }
                               // Create the submenu with the controllers
                               createSubMenu(controllers_text);
                               // Add actions for each controller
                               createSubMenuActions(controllers_text, sc_set_controller_);
                             }});

  main_menu_rows_.push_back({"Set Tracker", [this]() {
                               std::vector<std::string> trackers_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 trackers_text = uav_status_.trackers;
                               }
                               // Create the submenu with the trackers
                               createSubMenu(trackers_text);
                               // Add actions for each tracker
                               createSubMenuActions(trackers_text, sc_set_tracker_);
                             }});

  main_menu_rows_.push_back({"Set Estimator", [this]() {
                               std::vector<std::string> odometry_lat_sources_text;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 odometry_lat_sources_text = uav_status_.odom_estimators;
                               }
                               // Create the submenu with the odometry sources
                               createSubMenu(odometry_lat_sources_text);
                               // Add actions for each odometry source
                               createSubMenuActions(odometry_lat_sources_text, sc_set_estimator_);
                             }});

  for (const auto &rows : main_menu_rows_) {
    main_menu_text_.push_back(rows.label);
  }

  mrs_uav_status::tui::StatusWindow menu(1, 32, main_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* setupGotoMenu() //{ */

void Status::setupGotoMenu() {
  std::string odom_frame;

  {
    std::scoped_lock lock(mutex_status_msg_);
    odom_frame = uav_status_.odom_frame;
  }

  goto_menu_inputs_.clear();
  goto_menu_text_.clear();
  goto_menu_text_.push_back(" X:                ");
  goto_menu_text_.push_back(" Y:                ");
  goto_menu_text_.push_back(" Z:                ");
  goto_menu_text_.push_back(" hdg:              ");
  goto_menu_text_.push_back(" " + odom_frame + " ");

  mrs_uav_status::tui::StatusWindow menu(1, 32, goto_menu_text_);
  menu_vec_.push_back(menu);

  for (int i = 0; i < 4; i++) {
    mrs_uav_status::tui::ControlBar tmpbox(8, menu.getWin(), goto_double_vec_[i]);
    goto_menu_inputs_.push_back(tmpbox);
  }
}

//}

/* setupDisplayMenu() //{ */

void Status::setupDisplayMenu() {
  setupDisplayText();

  mrs_uav_status::tui::StatusWindow menu(1, 32, display_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* setupDisplayText() //{ */

void Status::setupDisplayText() {
  display_menu_text_.clear();

  char                     command[50] = "tmux list-windows | cut -d' ' -f-2";
  std::string              response    = utils::callTerminal(command);
  std::vector<std::string> results;
  results = mrs_uav_status::utils::splitByChar(response, '\n');

  const bool skip_last = !results.empty() && results.back().empty();
  const auto end_index = skip_last ? results.size() - 1 : results.size();
  for (size_t i = 0; i < end_index; i++) {
    display_menu_text_.push_back("[ ] " + results[i]);
  }

  for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
    display_menu_text_[selected_tmux_window_[i]][1] = '*';
  }
}

//}

//}

} // namespace mrs_uav_status

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mrs_uav_status::Status>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
