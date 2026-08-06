#pragma once
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <memory>

// --- Internal Package Includes ---
#include <mrs_uav_status/tui/system_info.hpp>
#include <mrs_uav_status/tui/control_bar.hpp>
#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/tui/status_window.hpp>
#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/tui/colors.hpp>

// <curses.h> (transitively included above by the TUI helpers) defines OK as a
// preprocessor macro (0), colliding with mrs_msgs/SensorStatus::OK below.
#ifdef OK
#undef OK
#endif

#include <mrs_uav_status/utils/helpers.hpp>
#include <mrs_uav_status/utils/string_info.hpp>
#include <mrs_uav_status/utils/terminal.hpp>

#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/velocity_reference_stamped_srv.hpp>
#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>

namespace mrs_uav_status::tui
{

// Owns the ncurses windows, renders them from the latest received diagnostics, and drives the
// menus/goto/remote-control key handling. Status pushes messages in via the on*() setters and
// drives rendering/input via the rest of this public API.
class TUI {
public:
  struct TUIParams
  {
    std::string              uav_name;
    std::string              colorscheme;
    bool                     colorblind_mode;
    bool                     start_minimized;
    std::string              display_config_filename;
    std::string              turbo_remote_constraints;
    std::vector<std::string> service_list;
    std::vector<double>      goto_values;
  };

  // Sets up the default panes and the goto/service-call service clients (parsing params.service_list
  // into service_entries_).
  TUI(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_sc, const TUI::TUIParams &params);

  // | --------------------- Data push (thread-safe) --------------------- |
  // Each stores msg into its last_*_ snapshot under mutex_status_msg_, for the next render to read.
  void onGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo &msg);
  void onStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo &msg);
  void onControlInfo(const mrs_msgs::msg::ControlInfo &msg);
  void onCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo &msg);
  void onUavInfo(const mrs_msgs::msg::UavInfo &msg);
  void onSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo &msg);
  void onUavState(const mrs_msgs::msg::State &msg);
  // Parses an optional "-id <key>" / "-p" (persistent) preamble, then dedupes-or-appends the
  // remaining text into string_info_vec_ (shown in the GNSS & strings pane).
  void onString(const std_msgs::msg::String &msg);

  // Pushed once per render tick from Status; true if the topic has ever arrived and hasn't timed out.
  void setDataFreshness(bool general_robot_info, bool collision_avoidance_info, bool uav_info, bool system_health_info, bool state_estimation_info);

  // | --------------------- Window lifecycle ------------------- |
  // (Re)creates all ncurses windows, sized/positioned for the current minimized/full layout, and
  // reapplies the colorscheme.
  void setupWindows();
  // If the terminal size changed and is wide enough, resizes the ncurses term and calls setupWindows(). Returns whether it resized.
  bool resize();
  // Queries tmux for the current pane size; returns whether it changed since the last call.
  bool updateTermSize();
  // Flips between minimized and full layout (next setupWindows() picks it up).
  void toggleMini();
  // Flips the help overlay on/off.
  void toggleHelp();
  bool isMini() const {
    return params_.start_minimized;
  }
  // True if the control manager reports flying_normally (gates remote mode / turbo remote).
  bool isFlyingNormally();
  void refreshTopBar();
  void setRemoteMode(bool in_remote_mode);

  // | --------------------- Window Handlers -------------------- |
  // Redraws the windows that need to update every tick: top bar, tmux/help overlay, UAV state box.
  void renderFast();
  // Redraws the throttled windows (called at update_rate_slow): counters/pruning, HW API, control manager, pane, general info.
  void renderSlow();
  // Draws position/heading, commanded-vs-estimated error, and current estimator names.
  void uavStateHandler();
  // Draws armed state, flight mode, battery, thrust, mass estimate, and GNSS/magnetometer readouts.
  void hwApiStateHandler();
  // Draws CPU load/frequency, RAM, and disk space.
  void generalInfoHandler();
  // Renders the currently selected cycleable pane, auto-switching to one whose wants_focus() just turned true.
  void paneHandler();
  // Draws the active controller/tracker/gains/constraints and callback/goal state.
  void controlManagerHandler();
  // Draws the top status bar: UAV name/type, collision-avoidance state, time-of-flight, hotkey hints.
  void topLineHandler();

  // Cycle the preset panel to the next preset (bound to the 'p' key).
  void cyclePanes();

  // Jump the preset panel directly to preset idx (bound to number keys). No-op if out of range.
  void selectPane(std::size_t idx);

  // Advances the 3-way rotation used to cycle which estimator name (hor/ver/hdg) uavStateHandler() shows.
  void tickSlowCounter();

  // Evict non-persistent display_string entries older than 10s. Runs every slow tick, independent of the currently selected pane.
  void pruneStrings();

  // | --------------------- Bottom-window helpers --------------- |
  // Clears the bottom window 3s after the last renderServiceResult(), so results don't linger forever.
  void blankBottomWindow();
  void refreshBottomWindow();
  // Prints a service call's success/failure message and marks the clear-time for blankBottomWindow().
  void renderServiceResult(bool success, const std::string &msg);

  // | ------------------- Menu (public entry) ------------------- |
  // Builds the top-level menu: per-service actions, toggle output, and set constraints/gains/controller/tracker/estimator submenus.
  void setupMainMenu();
  // Builds the goto menu's X/Y/Z/heading input boxes, seeded from the last-entered values.
  void setupGotoMenu();
  // Builds the tmux-window picker menu from the current tmux window list.
  void setupDisplayMenu();
  // Drives main-menu/submenu navigation; on Enter runs the selected action. Returns true when the whole menu should close.
  bool mainMenuHandler(int key);
  // Drives the 4 numeric input boxes; on Enter calls the goto-reference service. Returns true when done.
  bool gotoMenuHandler(int key);
  // Toggles a tmux window's selection (max MAX_SELECTED_TMUX_WINDOWS) and persists the choice to disk. Returns true when done.
  bool displayMenuHandler(int key);
  void clearMenus();
  // Reads the persisted tmux window selection from display_config_filename, if it exists.
  void loadDisplayConfig();
  // Draws the bottom debug window: remote-mode help, the selected tmux panes, or the keybinding help.
  void renderTmuxOrHelp();
  void refreshAfterMenu();

  // | -------------------------- Remote -------------------------- |
  // Dispatches one keypress in remote mode: 'T' toggles turbo, 'G' toggles local/global frame, else flies/hovers.
  void remoteHandler(int key);
  // Resets remote_hover_ when entering remote mode.
  void enterRemoteMode();

private:
  // | ------------------------- ROS Core ----------------------- |
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;


  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>         sc_goto_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv> sc_velocity_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_constraints_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_gains_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_estimator_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                     sc_hover_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>                     sc_toggle_output_;

  // Holds one service entry and its associated client handler.
  struct ServiceEntry
  {
    std::string                                           display_name;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> client;
  };

  std::vector<ServiceEntry> service_entries_;

  // | ----------------------- UAV status snapshot --------------- |
  // All last_*_ snapshots are guarded by this single mutex.
  std::mutex                            mutex_status_msg_;
  mrs_msgs::msg::GeneralRobotInfo       last_general_robot_info_;
  mrs_msgs::msg::StateEstimationInfo    last_state_estimation_info_;
  mrs_msgs::msg::ControlInfo            last_control_info_;
  mrs_msgs::msg::CollisionAvoidanceInfo last_collision_avoidance_info_;
  mrs_msgs::msg::UavInfo                last_uav_info_;
  mrs_msgs::msg::SystemHealthInfo       last_system_health_info_;
  mrs_msgs::msg::State                  last_uav_state_;

  // Custom strings published via std_msgs/String. Each entry tracks its own
  // freshness — entries older than 10 s are pruned in pruneStrings() (called
  // every slow tick, independent of the selected pane) unless marked
  // persistent (`-p` flag). Deduped by id (parsed from `-id <key>`).
  std::vector<utils::StringInfo> string_info_vec_;

  // | -------------------- Panes ---------------- |
  // The pane box cycles through pluggable panes ('p' key). To add a pane,
  // push a Pane in setupPanes(): give it a title, a render callback that
  // calls drawPaneChrome(win) itself (for the box + tab bar) before drawing
  // its own content rows, and optionally wants_focus() to auto-switch to it
  // when it has something important to show.
  struct Pane
  {
    std::string                      title;
    std::function<void(WINDOW *win)> render;
    std::function<bool()>            wants_focus; // optional; may be nullptr
  };
  std::vector<Pane> panes_;
  std::size_t       pane_idx_ = 0;
  std::vector<bool> pane_focus_prev_; // per-preset wants_focus() last state

  // Populates panes_ with the 4 built-in panes (Sensors, ROS Node CPU, GNSS & strings, Problems & errors).
  void setupPanes();
  int  drawPaneChrome(WINDOW *win); // box + title-in-border; returns first content row
  // Lists problems_preventing_start/errors from GeneralRobotInfo.
  void renderProblemsPane(WINDOW *win);
  // Lists available_sensors from SystemHealthInfo, worst-severity first.
  void renderSensorsPane(WINDOW *win);
  // Lists per-node CPU load from SystemHealthInfo, highest first.
  void renderNodeCpuPane(WINDOW *win);
  // Shows GNSS fix/accuracy and the current display_string entries.
  void renderStringsGnssPane(WINDOW *win);

  TUIParams params_;
  bool      light_scheme_   = false;
  bool      help_active_    = false;
  bool      in_remote_mode_ = false;

  // Holds one menu entry: its label, and an on_open action run when it's selected -- used for both
  // the main menu (where on_open typically builds a submenu) and submenus (where it typically calls
  // a service). Used by both main_menu_rows_ and sub_menu_rows_ below.
  struct MenuRow
  {
    std::string           label;
    std::function<void()> on_open;
  };

  std::vector<MenuRow> main_menu_rows_;
  std::vector<MenuRow> sub_menu_rows_;

  // | -------------------- Remote (private helpers) ------------ |
  // Sends one velocity reference, in the world frame if remote_global_ else the fcu_untilted frame.
  void remoteModeFly(const mrs_msgs::msg::VelocityReference &velocity_reference);
  // Draws the REMOTE/LOCAL-GLOBAL/TURBO banner over the top bar.
  void drawRemoteBanner(WINDOW *win);
  // Maps a keypress to a velocity command (wasd/hjkl/arrows for xy, r/f for z, q/e for heading); any
  // other key triggers hover() if a motion command was previously sent.
  void handleRemoteMotion(int key);
  // Toggles turbo_remote_constraints on/off via the set-constraints service, remembering the previous set.
  void toggleTurboRemote();

  bool        remote_hover_  = false;
  bool        turbo_remote_  = false;
  bool        remote_global_ = false;
  std::string old_constraints_;

  // | ------------------- Menu (private helpers) --------------- |
  static bool isValidMenuIndex(int index, size_t container_size);
  // Creates the submenu window next to the main menu, listing submenu_entries.
  void createSubMenu(std::vector<std::string> &submenu_entries);
  // Populates sub_menu_rows_ with one action per entry: calling it synchronously and rendering the
  // response via renderServiceResult(). Overloaded per service type; the Trigger/SetBool overloads
  // treat a "CANCEL" entry as a no-op instead of calling the service.
  void createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<mrs_msgs::srv::String> &service_client);
  void createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> &service_client);
  void createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> &service_client);

  // | ---------------------- TMUX & Misc ----------------------- |
  std::vector<int> selected_tmux_window_;
  std::string      session_name_;
  const int        MAX_SELECTED_TMUX_WINDOWS = 2;
  int              terminal_cols_ = 0, terminal_lines_ = 0;


  // Rebuilds display_menu_text_ from tmux's current window list, marking previously-selected windows.
  void setupDisplayText();

  int          estimator_display_counter_ = 0;
  bool         increment_counter_         = false;
  rclcpp::Time bottom_window_clear_time_;

  // Per-topic freshness, pushed by Status via setDataFreshness().
  bool have_general_robot_info_       = false;
  bool have_collision_avoidance_info_ = false;
  bool have_uav_info_                 = false;
  bool have_system_health_info_       = false;
  bool have_state_estimation_info_    = false;


  // | ---------------------- Window Pointers ------------------- |
  // RAII wrapper for ncurses windows — delwin() called on destruction.
  struct WindowDeleter
  {
    void operator()(WINDOW *w) const noexcept {
      if (w)
        delwin(w);
    }
  };

  using WindowPtr = std::unique_ptr<WINDOW, WindowDeleter>;

  WindowPtr uav_state_window_;
  WindowPtr control_manager_window_;
  WindowPtr hw_api_state_window_;
  WindowPtr top_bar_window_;
  WindowPtr bottom_window_;
  WindowPtr pane_window_; // cycleable panes (top-right): system detail / node CPU / GNSS / problems
  WindowPtr general_info_window_;
  WindowPtr debug_window_;
  WindowPtr sub_tmux_window_1_;
  WindowPtr sub_tmux_window_2_;

  // | ----------------------- Data Storage --------------------- |
  std::vector<tui::StatusWindow> menu_vec_;
  std::vector<tui::StatusWindow> submenu_vec_;
  std::vector<tui::ControlBar>   goto_menu_inputs_;

  std::vector<std::string> service_input_vec_;
  std::vector<std::string> main_menu_text_;
  std::vector<std::string> display_menu_text_;
  std::vector<std::string> goto_menu_text_;
  std::vector<double>      goto_double_vec_;
};

} // namespace mrs_uav_status::tui
