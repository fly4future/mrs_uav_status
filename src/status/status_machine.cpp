#include <mrs_uav_status/status/status_machine.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status::status
{

void StatusMachine::handleTick(const TickInput &input, tui::TuiActions &tui) {

  tui.setDataFreshness(input.freshness.general_robot_info, input.freshness.collision_avoidance_info, input.freshness.uav_info,
                       input.freshness.system_health_info, input.freshness.state_estimation_info);

  switch (state_) {

  case StatusState::STANDARD: {
    switch (input.key) {

    case 'R': {
      if (tui.isFlyingNormally()) {
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
      tui.selectPane(static_cast<std::size_t>(input.key - '1'));
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
    tui.remoteHandler(input.key);
    if (input.key == 'R' || input.key == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {
      tui.setRemoteMode(false);
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::MAIN_MENU: {
    tui.flushInput();
    if (tui.mainMenuHandler(input.key)) {
      tui.clearMenus();
      tui.refreshAfterMenu();
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::GOTO_MENU: {
    tui.flushInput();
    if (tui.gotoMenuHandler(input.key)) {
      tui.clearMenus();
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::DISPLAY_MENU: {
    tui.flushInput();
    if (tui.displayMenuHandler(input.key)) {
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
