#include <mrs_uav_status/status.hpp>

namespace
{

mrs_uav_status::status::GeneralRobotInfoData toData(const mrs_msgs::msg::GeneralRobotInfo &msg) {
  return {
      .robot_name                = msg.robot_name,
      .robot_type                = msg.robot_type,
      .ready_to_start            = msg.ready_to_start,
      .problems_preventing_start = msg.problems_preventing_start,
      .errors                    = msg.errors,
      .battery_state             = {.voltage = msg.battery_state.voltage, .current = msg.battery_state.current, .wh_drained = msg.battery_state.wh_drained},
  };
}

mrs_uav_status::status::StateEstimationInfoData toData(const mrs_msgs::msg::StateEstimationInfo &msg) {
  return {
      .frame_id              = msg.header.frame_id,
      .pos_x                 = msg.local_pose.position.x,
      .pos_y                 = msg.local_pose.position.y,
      .pos_z                 = msg.local_pose.position.z,
      .heading               = msg.local_pose.heading,
      .current_estimator     = msg.current_estimator,
      .switchable_estimators = msg.switchable_estimators,
      .horizontal_estimator  = msg.horizontal_estimator,
      .vertical_estimator    = msg.vertical_estimator,
      .heading_estimator     = msg.heading_estimator,
      .agl_estimator         = msg.agl_estimator,
      .max_flight_z          = msg.max_flight_z,
  };
}

mrs_uav_status::status::ControlInfoData toData(const mrs_msgs::msg::ControlInfo &msg) {
  return {
      .active_controller     = msg.active_controller,
      .available_controllers = msg.available_controllers,
      .active_gains          = msg.active_gains,
      .available_gains       = msg.available_gains,
      .active_tracker        = msg.active_tracker,
      .available_trackers    = msg.available_trackers,
      .active_constraints    = msg.active_constraints,
      .available_constraints = msg.available_constraints,
      .thrust                = msg.thrust,
      .cmd_pose_x            = msg.cmd_pose.position.x,
      .cmd_pose_y            = msg.cmd_pose.position.y,
      .cmd_pose_z            = msg.cmd_pose.position.z,
      .cmd_pose_heading      = msg.cmd_pose.heading,
      .flying_normally       = msg.flying_normally,
      .have_goal             = msg.have_goal,
      .tracking_trajectory   = msg.tracking_trajectory,
      .callbacks_enabled     = msg.callbacks_enabled,
  };
}

mrs_uav_status::status::CollisionAvoidanceInfoData toData(const mrs_msgs::msg::CollisionAvoidanceInfo &msg) {
  return {
      .collision_avoidance_enabled = msg.collision_avoidance_enabled,
      .avoiding_collision          = msg.avoiding_collision,
      .bumper_active               = msg.bumper_active,
      .num_other_robots_visible    = msg.other_robots_visible.size(),
  };
}

mrs_uav_status::status::UavInfoData toData(const mrs_msgs::msg::UavInfo &msg) {
  return {
      .flight_state    = msg.flight_state,
      .flight_duration = msg.flight_duration,
      .armed           = msg.armed,
      .offboard        = msg.offboard,
      .mass_nominal    = msg.mass_nominal,
      .mass_estimate   = msg.mass_estimate,
  };
}

mrs_uav_status::status::SystemHealthInfoData toData(const mrs_msgs::msg::SystemHealthInfo &msg) {
  mrs_uav_status::status::SystemHealthInfoData data;
  data.onboard_computer_info.cpu_load  = msg.onboard_computer_info.cpu_load;
  data.onboard_computer_info.cpu_ghz   = msg.onboard_computer_info.cpu_ghz;
  data.onboard_computer_info.free_ram  = msg.onboard_computer_info.free_ram;
  data.onboard_computer_info.total_ram = msg.onboard_computer_info.total_ram;
  data.onboard_computer_info.free_hdd  = msg.onboard_computer_info.free_hdd;
  for (const auto &n : msg.onboard_computer_info.node_cpu_loads) {
    data.onboard_computer_info.node_cpu_loads.push_back({.node_name = n.node_name, .cpu_load = n.cpu_load});
  }
  data.hw_api_rate           = msg.hw_api_rate;
  data.control_manager_rate  = msg.control_manager_rate;
  data.state_estimation_rate = msg.state_estimation_rate;
  for (const auto &s : msg.available_sensors) {
    mrs_uav_status::status::SensorStatusData sd;
    sd.type    = s.type;
    sd.name    = s.name;
    sd.rate    = s.rate;
    sd.level   = s.level;
    sd.message = s.message;
    for (const auto &d : s.details) {
      sd.details.push_back({.key = d.key, .value = d.value});
    }
    data.available_sensors.push_back(std::move(sd));
  }
  return data;
}

mrs_uav_status::status::StateData toData(const mrs_msgs::msg::State &msg) {
  return {.state = msg.state};
}

} // namespace

namespace mrs_uav_status
{

/* Status() //{ */

Status::Status() : Node("mrs_status_menu") {

  tui::TUI::initTerminal();

  initialize();
}

//}

/* ~Status() //{ */

Status::~Status() {
  timer_render_.reset();
  tui::TUI::shutdownTerminal();
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

  std::string                 pwd, colorscheme, turbo_remote_constraints, uav_name;
  double                      update_rate, update_rate_slow, resize_rate;
  Eigen::Matrix<double, 4, 1> goto_values_mat;
  bool                        colorblind_mode = false;
  bool                        start_minimized = false;

  param_loader.loadParam("pwd", pwd);
  param_loader.loadParam("colorscheme", colorscheme);
  param_loader.loadParam("uav_name", uav_name);
  param_loader.loadParam("mrs_uav_status/rates/update_rate", update_rate);
  param_loader.loadParam("mrs_uav_status/rates/update_rate_slow", update_rate_slow);
  param_loader.loadParam("mrs_uav_status/rates/resize_rate", resize_rate);
  param_loader.loadParam("mrs_uav_status/data_timeout", data_timeout_s_);
  param_loader.loadParam("mrs_uav_status/turbo_remote_constraints", turbo_remote_constraints);
  param_loader.loadParam("mrs_uav_status/display/colorblind_mode", colorblind_mode);
  param_loader.loadParam("mrs_uav_status/enable_profiler", _profiler_enabled_);
  param_loader.loadParam("mrs_uav_status/display/start_minimized", start_minimized);
  std::vector<std::string> service_list;
  param_loader.loadParam("mrs_uav_status/menu/service_list", service_list);
  param_loader.loadMatrixStatic("mrs_uav_status/menu/goto_values", goto_values_mat);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }
  RCLCPP_INFO(node_->get_logger(), "All params loaded!");

  const std::string         display_config_filename = pwd + "/.mrs_status_display_config~";
  const std::vector<double> goto_values(goto_values_mat.data(), goto_values_mat.data() + goto_values_mat.size());

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

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* timerRender() //{ */

void Status::timerRender() {

  if (!is_initialized_) {
    return;
  }

  // Resize before anything else draws; force a slow redraw right after so it isn't left blank.
  const rclcpp::Time now     = clock_->now();
  bool               resized = false;

  // hasMsg() guards against sim-time-near-zero at boot; the elapsed-time check catches mid-flight stalls.
  auto is_fresh = [&now, this](const auto &sh) { return sh.hasMsg() && (now - sh.lastMsgTime()).seconds() < data_timeout_s_; };
  tui_->setDataFreshness(is_fresh(sh_general_robot_info_), is_fresh(sh_collision_avoidance_info_), is_fresh(sh_uav_info_), is_fresh(sh_system_health_info_),
                         is_fresh(sh_state_estimation_info_));
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

  int key = tui_->pollKey();

  switch (state_) {

    /* STANDARD //{ */
  case StatusState::STANDARD: {

    switch (key) {

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
      tui_->selectPane(static_cast<size_t>(key - '1'));
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
      tui_->flushInput();
      break;
    }
    break;
  }
    //}

    /* REMOTE //{ */
  case StatusState::REMOTE: {
    tui_->flushInput();
    tui_->remoteHandler(key);
    if (key == 'R' || key == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {
      tui_->setRemoteMode(false);
      state_ = StatusState::STANDARD;
    }
    break;
  }
    //}

    /* MAIN_MENU //{ */
  case StatusState::MAIN_MENU: {
    tui_->flushInput();
    if (tui_->mainMenuHandler(key)) {
      tui_->clearMenus();
      tui_->refreshAfterMenu();
      state_ = StatusState::STANDARD;
    }
    break;
  }
    //}

    /* GOTO_MENU //{ */
  case StatusState::GOTO_MENU: {
    tui_->flushInput();
    if (tui_->gotoMenuHandler(key)) {
      tui_->clearMenus();
      state_ = StatusState::STANDARD;
    }
    break;
  }
    //}

    /* DISPLAY_MENU //{ */
  case StatusState::DISPLAY_MENU: {
    tui_->flushInput();
    if (tui_->displayMenuHandler(key)) {
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
  tui_->commitFrame();
}

//}

/* callbacks //{ */

void Status::callbackGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  tui_->onGeneralRobotInfo(toData(*msg));
}

void Status::callbackStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  tui_->onStateEstimationInfo(toData(*msg));
}

void Status::callbackControlInfo(const mrs_msgs::msg::ControlInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  tui_->onControlInfo(toData(*msg));
}

void Status::callbackCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  tui_->onCollisionAvoidanceInfo(toData(*msg));
}

void Status::callbackUavInfo(const mrs_msgs::msg::UavInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  tui_->onUavInfo(toData(*msg));
}

void Status::callbackSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  tui_->onSystemHealthInfo(toData(*msg));
}

void Status::callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  tui_->onUavState(toData(*msg));
}

void Status::callbackDisplayString(const std_msgs::msg::String::ConstSharedPtr msg) {
  if (!is_initialized_) {
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
