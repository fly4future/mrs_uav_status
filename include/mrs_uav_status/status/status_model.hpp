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

// The package's decision layer: owns the STANDARD/REMOTE/menu state machine and (from later
// tasks) every received message snapshot plus all menu/goto/remote logic. No ROS, no ncurses --
// it only ever drives an abstract tui::TuiActions, so tests can substitute a fake.
class StatusModel {
public:
  // Stores the per-topic freshness for this tick. Call once before every tick().
  void setFreshness(const Freshness &freshness);

  // Advances the state machine by one render tick: dispatches key through the current state and
  // drives tui accordingly. now_seconds is the node clock in plain seconds.
  void tick(double now_seconds, int key, tui::TuiActions &tui);

  StatusState state() const {
    return state_;
  }

private:
  StatusState state_ = StatusState::STANDARD;
  Freshness   freshness_;
};

} // namespace mrs_uav_status::status
