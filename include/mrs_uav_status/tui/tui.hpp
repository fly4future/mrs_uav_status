#pragma once
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <memory>

// --- Internal Package Includes ---
#include <mrs_uav_status/ros/topic_info.hpp>

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
#include <mrs_msgs/msg/custom_topic.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_msgs/msg/gimbal_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>

namespace mrs_uav_status::tui
{

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

  TUI(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_sc, const TUI::TUIParams &params);

  // | --------------------- Data push (thread-safe) --------------------- |
  void onGeneralRobotInfo(const mrs_msgs::msg::GeneralRobotInfo &msg);
  void onStateEstimationInfo(const mrs_msgs::msg::StateEstimationInfo &msg);
  void onControlInfo(const mrs_msgs::msg::ControlInfo &msg);
  void onCollisionAvoidanceInfo(const mrs_msgs::msg::CollisionAvoidanceInfo &msg);
  void onUavInfo(const mrs_msgs::msg::UavInfo &msg);
  void onSystemHealthInfo(const mrs_msgs::msg::SystemHealthInfo &msg);
  void onUavState(const mrs_msgs::msg::State &msg);
  void onString(const std_msgs::msg::String &msg);

  // | --------------------- Window lifecycle ------------------- |
  void setupWindows();
  void resize();
  bool updateTermSize();
  void toggleMini();
  void toggleHelp();
  bool isMini() const {
    return params_.start_minimized;
  }
  bool isFlyingNormally();
  void refreshTopBar();
  void setRemoteMode(bool in_remote_mode);

  // | --------------------- Window Handlers -------------------- |
  void uavStateHandler();
  void hwApiStateHandler();
  void generalInfoHandler();
  void genericTopicHandler();
  void paneHandler();
  void controlManagerHandler();
  void topLineHandler();

  /** @brief Cycle the preset panel to the next preset (bound to the 'p' key). */
  void cyclePanes();

  /** @brief Jump the preset panel directly to preset @p idx (bound to number keys). No-op if out of range. */
  void selectPane(std::size_t idx);

  void tickSlowCounter();

  // | --------------------- Bottom-window helpers --------------- |
  void blankBottomWindow();
  void refreshBottomWindow();
  void renderServiceResult(bool success, const std::string &msg);

  // | ------------------- Menu (public entry) ------------------- |
  void setupMainMenu();
  void setupGotoMenu();
  void setupDisplayMenu();
  bool mainMenuHandler(int key_in);
  bool gotoMenuHandler(int key_in);
  bool displayMenuHandler(int key_in);
  void clearMenus();
  void loadDisplayConfig();
  void renderTmuxOrHelp();
  void refreshAfterMenu();

  // | -------------------- Remote & Gimbal --------------------- |
  void remoteHandler(int key);
  void gimbalHandler(int key);
  void resetGimbalCommand();
  void enterRemoteMode();

private:
  // | ------------------------- ROS Core ----------------------- |
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;


  mrs_lib::PublisherHandler<mrs_msgs::msg::GimbalState>                     ph_gimbal_state_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>         sc_goto_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv> sc_velocity_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_constraints_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_gains_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                      sc_set_estimator_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                     sc_hover_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>                     sc_toggle_output_;

  /** @brief struct to hold service entries and their associated client handlers */
  struct ServiceEntry
  {
    std::string                                           display_name;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> client;
  };

  std::vector<ServiceEntry> service_entries_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

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
  // freshness — entries older than 10 s are pruned in stringHandler unless
  // marked persistent (`-p` flag). Deduped by id (parsed from `-id <key>`).
  std::vector<utils::StringInfo> string_info_vec_;

  // | -------------------- Panes ---------------- |
  // The pane box cycles through pluggable panes ('p' key). To add a pane,
  // push a Pane in setupPanes(): give it a title, a render
  // callback that draws content rows (the dispatcher handles the box + title),
  // and optionally wants_focus() to auto-switch to it when it has
  // something important to show.
  struct Pane
  {
    std::string                      title;
    std::function<void(WINDOW *win)> render;
    std::function<bool()>            wants_focus; // optional; may be nullptr
  };
  std::vector<Pane> panes_;
  std::size_t       pane_idx_ = 0;
  std::vector<bool> pane_focus_prev_; // per-preset wants_focus() last state

  void setupPanes();
  int  drawPaneChrome(WINDOW *win); // box + title-in-border; returns first content row
  void renderProblemsPane(WINDOW *win);
  void renderSensorsPane(WINDOW *win);
  void renderNodeCpuPane(WINDOW *win);
  void renderStringsGnssPane(WINDOW *win);

  /** @brief struct to hold menu entries and their associated actions */
  /*
  // This is used for both the main menu and submenus, where the on_open function defines what happens when the menu entry is selected
  // For the main menu, the on_open function typically creates a submenu with specific entries and actions. For the submenus, the on_open function typically
      - @label: the text displayed for the menu entry
      - @on_open: a function that is called when the menu entry is selected. This function can be used to create submenus or perform actions directly. It is
  defined as a std::function that takes no arguments and returns void, allowing for flexibility in the actions that can be performed when a menu entry is
  selected.
  */

  TUIParams params_;
  bool      _light_         = false;
  bool      help_active_    = false;
  bool      in_remote_mode_ = false;

  struct MenuRow
  {
    std::string           label;
    std::function<void()> on_open;
  };

  std::vector<MenuRow> main_menu_rows_;
  std::vector<MenuRow> sub_menu_rows_;

  // | -------------------- Remote (private helpers) ------------ |
  void remoteModeFly(const mrs_msgs::msg::VelocityReference &ref_in);
  void drawRemoteBanner(WINDOW *win);
  void handleRemoteMotion(int key);
  void toggleTurboRemote();

  bool                       remote_hover_  = false;
  bool                       turbo_remote_  = false;
  bool                       remote_global_ = false;
  std::string                old_constraints_;
  mrs_msgs::msg::GimbalState gimbal_command_;

  // | ------------------- Menu (private helpers) --------------- |
  static bool isValidMenuIndex(int index, size_t container_size);
  void        createSubMenu(std::vector<std::string> &submenu_entries);
  void        createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<mrs_msgs::srv::String> &service_client);
  void        createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> &service_client);
  void        createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> &service_client);

  // | ---------------------- TMUX & Misc ----------------------- |
  std::vector<int> selected_tmux_window_;
  std::string      session_name_;
  const int        MAX_SELECTED_TMUX_WINDOWS = 2;
  int              terminal_cols_ = 0, terminal_lines_ = 0;


  void setupDisplayText();

  long         last_gigas_                = 0;
  bool         have_data_                 = false;
  int          estimator_display_counter_ = 0;
  bool         increment_counter_         = false;
  rclcpp::Time last_time_got_data_;
  rclcpp::Time bottom_window_clear_time_;


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
  WindowPtr generic_topic_window_;
  WindowPtr pane_window_; // cycleable panes (top-right): system detail / node CPU / GNSS / problems
  WindowPtr general_info_window_;
  WindowPtr debug_window_;
  WindowPtr sub_tmux_window_1_;
  WindowPtr sub_tmux_window_2_;

  // | ----------------------- Data Storage --------------------- |
  std::vector<tui::StatusWindow> menu_vec_;
  std::vector<tui::StatusWindow> submenu_vec_;
  std::vector<tui::ControlBar>   goto_menu_inputs_;

  std::vector<TopicInfo>   string_topic_;
  std::vector<std::string> service_input_vec_;
  std::vector<std::string> main_menu_text_;
  std::vector<std::string> display_menu_text_;
  std::vector<std::string> goto_menu_text_;
  std::vector<double>      goto_double_vec_;
};

} // namespace mrs_uav_status::tui
