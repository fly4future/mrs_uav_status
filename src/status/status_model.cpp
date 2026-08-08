#include <mrs_uav_status/status/status_model.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status::status
{

void StatusModel::setFreshness(const Freshness &freshness) {
  std::scoped_lock lock(mutex_status_msg_);
  freshness_ = freshness;
}

void StatusModel::onGeneralRobotInfo(const GeneralRobotInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_general_robot_info_ = data;
}

void StatusModel::onStateEstimationInfo(const StateEstimationInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_state_estimation_info_ = data;
}

void StatusModel::onControlInfo(const ControlInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_control_info_ = data;
}

void StatusModel::onCollisionAvoidanceInfo(const CollisionAvoidanceInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_collision_avoidance_info_ = data;
}

void StatusModel::onUavInfo(const UavInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_uav_info_ = data;
}

void StatusModel::onSystemHealthInfo(const SystemHealthInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_system_health_info_ = data;
}

void StatusModel::onUavState(const StateData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_uav_state_ = data;
}

bool StatusModel::isFlyingNormally() const {
  std::scoped_lock lock(mutex_status_msg_);
  return last_control_info_.flying_normally;
}

RenderSnapshot StatusModel::snapshot(double now_seconds) const {
  std::scoped_lock lock(mutex_status_msg_);

  RenderSnapshot out;
  out.now_seconds              = now_seconds;
  out.freshness                = freshness_;
  out.general_robot_info       = last_general_robot_info_;
  out.state_estimation_info    = last_state_estimation_info_;
  out.control_info             = last_control_info_;
  out.collision_avoidance_info = last_collision_avoidance_info_;
  out.uav_info                 = last_uav_info_;
  out.system_health_info       = last_system_health_info_;
  out.uav_state                = last_uav_state_;
  out.border_status            = BorderStatus{
                 .avoiding_collision = last_collision_avoidance_info_.avoiding_collision,
                 .bumper_active      = last_collision_avoidance_info_.bumper_active,
                 .can_takeoff        = last_general_robot_info_.ready_to_start,
                 .null_tracker       = (last_control_info_.active_tracker == "NullTracker"),
  };
  return out;
}

void StatusModel::tick([[maybe_unused]] double now_seconds, int key, tui::TuiActions &tui) {

  switch (state_) {

  case StatusState::STANDARD: {
    switch (key) {

    case 'R': {
      if (isFlyingNormally()) {
        tui.enterRemoteMode();
        tui.setRemoteMode(true);
        state_ = StatusState::REMOTE;
      }
      break;
    }

    case 'm':
      tui.setupMainMenu();
      state_ = StatusState::MAIN_MENU;
      break;

    case 'g':
      tui.setupGotoMenu();
      state_ = StatusState::GOTO_MENU;
      break;

    case 'h':
      tui.toggleHelp();
      break;

    case 'p':
      tui.cyclePanes();
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
      tui.selectPane(static_cast<std::size_t>(key - '1'));
      break;

    case 'M':
      tui.toggleMini();
      tui.setupWindows();
      tui.renderFast();
      tui.renderSlow();
      break;

    case 'D':
      tui.setupDisplayMenu();
      state_ = StatusState::DISPLAY_MENU;
      break;

    default:
      tui.flushInput();
      break;
    }
    break;
  }

  case StatusState::REMOTE: {
    tui.flushInput();
    tui.remoteHandler(key);
    if (key == 'R' || key == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {
      tui.setRemoteMode(false);
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::MAIN_MENU: {
    tui.flushInput();
    if (tui.mainMenuHandler(key)) {
      tui.clearMenus();
      tui.refreshAfterMenu();
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::GOTO_MENU: {
    tui.flushInput();
    if (tui.gotoMenuHandler(key)) {
      tui.clearMenus();
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::DISPLAY_MENU: {
    tui.flushInput();
    if (tui.displayMenuHandler(key)) {
      tui.clearMenus();
      state_ = StatusState::STANDARD;
    }
    break;
  }
  }

  if (state_ != StatusState::MAIN_MENU && state_ != StatusState::GOTO_MENU && state_ != StatusState::DISPLAY_MENU) {
    tui.refreshBottomWindow();
  }
}

} // namespace mrs_uav_status::status
