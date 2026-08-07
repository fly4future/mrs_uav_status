#pragma once

#include <mrs_uav_status/status/data_types.hpp>
#include <mrs_uav_status/tui/tui_actions.hpp>

namespace mrs_uav_status::status
{

enum class StatusState
{
  STANDARD,
  REMOTE,
  MAIN_MENU,
  GOTO_MENU,
  DISPLAY_MENU
};

// Owns the STANDARD/REMOTE/menu FSM. No ROS Node, no ncurses, no messages -- takes a plain
// TickInput and drives a tui::TuiActions each render tick. rclcpp::Time (inside TickInput) is
// the one allowed ROS dependency, for parity with ros_wrapper's clock; StatusMachine itself
// does not read TickInput::now today.
class StatusMachine {
public:
  void handleTick(const TickInput &input, tui::TuiActions &tui);

  StatusState state() const {
    return state_;
  }

private:
  StatusState state_ = StatusState::STANDARD;
};

} // namespace mrs_uav_status::status
