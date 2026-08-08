#pragma once

#include <mutex>

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

  // | --------------------- Data push (thread-safe) --------------------- |
  // Called from ROS subscriber threads; each stores data into its last_*_ snapshot under
  // mutex_status_msg_ for the next snapshot() to read.
  void onGeneralRobotInfo(const GeneralRobotInfoData &data);
  void onStateEstimationInfo(const StateEstimationInfoData &data);
  void onControlInfo(const ControlInfoData &data);
  void onCollisionAvoidanceInfo(const CollisionAvoidanceInfoData &data);
  void onUavInfo(const UavInfoData &data);
  void onSystemHealthInfo(const SystemHealthInfoData &data);
  void onUavState(const StateData &data);

  // Deep-copies everything the renderer needs, under mutex_status_msg_. Call once per fast tick,
  // after setFreshness() and before any TUI render call.
  RenderSnapshot snapshot(double now_seconds) const;

private:
  // True if ControlInfo reports flying_normally (gates remote mode and turbo remote).
  bool isFlyingNormally() const;

  StatusState state_ = StatusState::STANDARD;

  // Threading invariant: last_*_ members and freshness_ below are written from ROS subscriber
  // threads (the on*() setters) and read by snapshot(), which deep-copies them out under this
  // lock exactly once per fast tick. tui::TUI::snapshot_ (the copy snapshot() produces) is then
  // written and read only from the render thread, so TUI itself needs no mutex for this data.
  mutable std::mutex         mutex_status_msg_;
  Freshness                  freshness_;
  GeneralRobotInfoData       last_general_robot_info_;
  StateEstimationInfoData    last_state_estimation_info_;
  ControlInfoData            last_control_info_;
  CollisionAvoidanceInfoData last_collision_avoidance_info_;
  UavInfoData                last_uav_info_;
  SystemHealthInfoData       last_system_health_info_;
  StateData                  last_uav_state_;
};

} // namespace mrs_uav_status::status
