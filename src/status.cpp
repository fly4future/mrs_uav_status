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

/* ~Status() //{ */

Status::~Status() {
  timer_render_.reset();
  endwin();
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

  std::string         pwd, colorscheme, turbo_remote_constraints, uav_name;
  double              update_rate, update_rate_slow, resize_rate;
  std::vector<double> goto_values;
  bool                colorblind_mode = false;
  bool                start_minimized = false;

  param_loader.loadParam("pwd", pwd);
  param_loader.loadParam("colorscheme", colorscheme);
  param_loader.loadParam("uav_name", uav_name);
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

  const tui::TUI::TUIParams tui_params{
      .uav_name                 = uav_name,
      .colorscheme              = colorscheme,
      .colorblind_mode          = colorblind_mode,
      .start_minimized          = start_minimized,
      .display_config_filename  = display_config_filename,
      .turbo_remote_constraints = turbo_remote_constraints,
      .service_list             = service_list,
      .goto_values              = goto_values,
  };

  tui_ = std::make_unique<tui::TUI>(node_, cbkgrp_sc_, tui_params);
  tui_->updateTermSize();
  tui_->setupWindows();
  tui_->loadDisplayConfig();

  // | ------------------------- Timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;
  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  timer_render_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(update_rate, clock_), std::bind(&Status::timerRender, this));

  slow_period_       = rclcpp::Duration::from_seconds(1.0 / update_rate_slow);
  resize_period_     = rclcpp::Duration::from_seconds(1.0 / resize_rate);
  last_slow_run_     = rclcpp::Time(0, 0, clock_->get_clock_type());
  last_resize_check_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | ------------------------ Subscribers ------------------------ |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_general_robot_info_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>(shopts, "~/general_robot_info_in", &Status::callbackGeneralRobotInfo, this);
  sh_state_estimation_info_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>(shopts, "~/state_estimation_info_in", &Status::callbackStateEstimationInfo, this);
  sh_control_info_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>(shopts, "~/control_info_in", &Status::callbackControlInfo, this);
  sh_collision_avoidance_info_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo>(shopts, "~/collision_avoidance_info_in", &Status::callbackCollisionAvoidanceInfo, this);
  sh_uav_info_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>(shopts, "~/uav_info_in", &Status::callbackUavInfo, this);
  sh_system_health_info_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>(shopts, "~/system_health_info_in", &Status::callbackSystemHealthInfo, this);
  sh_uav_state_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::State>(shopts, "~/uav_state_in", &Status::callbackUavState, this);
  // Custom-display string topic — anything published here lands in the TUI's
  // Strings window. Supports "-id <key> -p <space-separated text>" preamble.
  sh_display_string_ = mrs_lib::SubscriberHandler<std_msgs::msg::String>(shopts, "~/display_string_in", &Status::callbackDisplayString, this);

  profiler_ = mrs_lib::Profiler(node_, "Status", _profiler_enabled_);

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* timerRender() //{ */

void Status::timerRender() {

  if (!initialized_) {
    return;
  }

  // Resize before anything else draws; force a slow redraw right after so it isn't left blank.
  const rclcpp::Time now     = clock_->now();
  bool               resized = false;
  if (now - last_resize_check_ >= resize_period_) {
    last_resize_check_                = now;
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("resize");
    resized                           = tui_->resize();
  }
  if (resized || now - last_slow_run_ >= slow_period_) {
    last_slow_run_                    = now;
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("renderSlow");
    tui_->renderSlow();
  }

  {
    mrs_lib::Routine profiler_routine = profiler_.createRoutine("renderFast");
    tui_->renderFast();
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
        tui_->setRemoteMode(true);
        state_ = StatusState::REMOTE;
      }
      break;
    }

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

    case 'p':
      tui_->cyclePanes();
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      tui_->selectPane(static_cast<size_t>(key_in - '1'));
      break;

    case 'M':
      tui_->toggleMini();
      tui_->setupWindows();
      tui_->renderFast();
      tui_->renderSlow();
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
      tui_->setRemoteMode(false);
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

/* callbacks //{ */

void Status::callbackGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onGeneralRobotInfo(*msg);
}

void Status::callbackStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onStateEstimationInfo(*msg);
}

void Status::callbackControlInfo(const mrs_msgs::msg::ControlInfo::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onControlInfo(*msg);
}

void Status::callbackCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onCollisionAvoidanceInfo(*msg);
}

void Status::callbackUavInfo(const mrs_msgs::msg::UavInfo::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onUavInfo(*msg);
}

void Status::callbackSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onSystemHealthInfo(*msg);
}

void Status::callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onUavState(*msg);
}

void Status::callbackDisplayString(const std_msgs::msg::String::ConstSharedPtr msg) {
  if (!initialized_) {
    return;
  }
  tui_->onString(*msg);
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
