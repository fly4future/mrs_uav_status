#include <mrs_uav_status/status.hpp>

namespace mrs_uav_status
{

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
  curs_set(0);
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

  std::string pwd, colorscheme, turbo_remote_constraints;
  double      update_rate, update_rate_slow, resize_rate;
  std::vector<double> goto_values; 
  bool        colorblind_mode = false;
  bool        start_minimized = false;

  param_loader.loadParam("pwd", pwd);
  param_loader.loadParam("colorscheme", colorscheme);
  param_loader.loadParam("mrs_uav_status/update_rate", update_rate);
  param_loader.loadParam("mrs_uav_status/update_rate_slow", update_rate_slow);
  param_loader.loadParam("mrs_uav_status/resize_rate", resize_rate);
  param_loader.loadParam("mrs_uav_status/turbo_remote_constraints", turbo_remote_constraints);
  param_loader.loadParam("mrs_uav_status/colorblind_mode", colorblind_mode);
  param_loader.loadParam("mrs_uav_status/enable_profiler", _profiler_enabled_);
  param_loader.loadParam("mrs_uav_status/start_minimized", start_minimized);
  std::vector<std::string> service_list;
  param_loader.loadParam("mrs_uav_status/service_list", service_list);
  param_loader.loadParam("mrs_uav_status/goto_values", goto_values);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }
  RCLCPP_INFO(node_->get_logger(), "All params loaded!");

  const std::string display_config_filename = pwd + "/.mrs_status_display_config~";

  tui_ = std::make_unique<tui::TUI>(node_, cbkgrp_sc_, colorscheme, colorblind_mode, start_minimized, display_config_filename, turbo_remote_constraints, service_list, goto_values);
  tui_->updateTermSize();
  tui_->setupWindows();
  tui_->loadDisplayConfig();

  // | ------------------------- Timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;
  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  timer_status_fast_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(update_rate, clock_), std::bind(&Status::timerStatusFast, this));
  timer_status_slow_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(update_rate_slow, clock_), std::bind(&Status::timerStatusSlow, this));
  timer_resize_      = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(resize_rate, clock_), std::bind(&Status::timerResize, this));

  // | ------------------------ Subscribers ------------------------ |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_uav_status_       = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>(shopts, "~/uav_status_in", &Status::callbackUavStatus, this);
  sh_uav_status_short_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatusShort>(shopts, "~/uav_status_short_in", &Status::callbackUavStatusShort, this);

  profiler_ = mrs_lib::Profiler(node_, "Status", _profiler_enabled_);

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* timerResize() //{ */

void Status::timerResize() {
  if (!initialized_) {
    return;
  }
  tui_->resize();
}

//}

/* timerStatusFast() //{ */

void Status::timerStatusFast() {

  if (!initialized_) {
    return;
  }

  tui_->topLineHandler();
  tui_->renderTmuxOrHelp();

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("uavStateHandler");
    tui_->uavStateHandler();
  }

  tui_->blankBottomWindow();

  int key_in = getch();

  switch (state_) {

    /* STANDARD //{ */
  case StatusState::STANDARD: {

    switch (key_in) {

    case 'R': {
      if (tui_->isFlyingNormally()) {
        tui_->enterRemoteMode();
        state_ = StatusState::REMOTE;
      }
      break;
    }

    case 'G':
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
      tui_->toggleHelp();
      break;

    case 'M':
      tui_->toggleMini();
      tui_->setupWindows();
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
    break;
  }
    //}

    /* REMOTE //{ */
  case StatusState::REMOTE: {
    flushinp();
    tui_->remoteHandler(key_in);
    if (key_in == 'R' || key_in == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {
      state_ = StatusState::STANDARD;
    }
    break;
  }
    //}

    /* GIMBAL //{ */
  case StatusState::GIMBAL: {
    flushinp();
    tui_->gimbalHandler(key_in);
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
      tui_->refreshAfterMenu();
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
    tui_->refreshBottomWindow();
  }
  tui_->refreshTopBar();
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
    tui_->hwApiStateHandler();
  }
  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("controlManagerHandler");
    tui_->controlManagerHandler();
  }
  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("genericTopicHandler");
    tui_->genericTopicHandler();
  }
  if (!tui_->isMini()) {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("nodeStatsHandler");
    tui_->nodeStatsHandler();
  }
  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("stringHandler");
    tui_->stringHandler();
  }
  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("generalInfoHandler");
    tui_->generalInfoHandler();
  }
}

//}

/* callbacks //{ */

void Status::callbackUavStatus(const mrs_msgs::msg::UavStatus::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onUavStatus(*msg);
}

void Status::callbackUavStatusShort(const mrs_msgs::msg::UavStatusShort::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onUavStatusShort(*msg);
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
