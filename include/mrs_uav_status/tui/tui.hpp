#pragma once
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <memory>

// --- Internal Package Includes ---
#include <mrs_uav_status/ros/service.hpp>
#include <mrs_uav_status/ros/topic_info.hpp>

#include <mrs_uav_status/tui/system_info.hpp>
#include <mrs_uav_status/tui/control_bar.hpp>
#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/tui/status_window.hpp>
#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/tui/colors.hpp>
#include <mrs_uav_status/utils/split.hpp>
#include <mrs_uav_status/utils/terminal.hpp>

#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/uav_status_short.hpp>
#include <mrs_msgs/msg/gimbal_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>

namespace mrs_uav_status::tui
{

class TUI {
public:
  TUI(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_sc, const std::string &colorscheme, bool colorblind_mode, bool minimized_mode,
      const std::string &display_config_filename, const std::string &turbo_remote_constraints);

  // | --------------------- Data push (thread-safe) --------------------- |
  void onUavStatus(const mrs_msgs::msg::UavStatus &msg);
  void onUavStatusShort(const mrs_msgs::msg::UavStatusShort &msg);

  // | --------------------- Window lifecycle ------------------- |
  void setupWindows();
  void resize();
  bool updateTermSize();
  void toggleMini();
  void toggleHelp();
  bool isMini() const {
    return mini_;
  }
  bool isFlyingNormally();
  void refreshTopBar();

  // | --------------------- Window Handlers -------------------- |
  void stringHandler();
  void uavStateHandler();
  void nodeStatsHandler();
  void hwApiStateHandler();
  void generalInfoHandler();
  void genericTopicHandler();
  void controlManagerHandler();
  void topLineHandler();

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
  std::string _colorscheme_;
  std::string _display_config_filename_;
  std::string _turbo_remote_constraints_;
  bool        _colorblind_mode_;
  bool        _light_      = false;
  bool        mini_        = false;
  bool        help_active_ = false;

  // | ------------------------- ROS Core ----------------------- |
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  mrs_lib::PublisherHandler<mrs_msgs::msg::GimbalState>             ph_gimbal_state_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sc_goto_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_constraints_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_gains_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sc_set_estimator_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sc_hover_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ----------------------- UAV status snapshot --------------- |
  std::mutex               mutex_status_msg_;
  mrs_msgs::msg::UavStatus uav_status_;

  /** @brief struct to hold menu entries and their associated actions */
  /*
  // This is used for both the main menu and submenus, where the on_open function defines what happens when the menu entry is selected
  // For the main menu, the on_open function typically creates a submenu with specific entries and actions. For the submenus, the on_open function typically
      - @label: the text displayed for the menu entry
      - @on_open: a function that is called when the menu entry is selected. This function can be used to create submenus or perform actions directly. It is
  defined as a std::function that takes no arguments and returns void, allowing for flexibility in the actions that can be performed when a menu entry is
  selected.
  */
  struct MenuRow
  {
    std::string           label;
    std::function<void()> on_open;
  };

  std::vector<MenuRow> main_menu_rows_;
  std::vector<MenuRow> sub_menu_rows_;

  // | -------------------- Remote (private helpers) ------------ |
  void remoteModeFly(const mrs_msgs::msg::Reference &ref_in);
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

  // | ---------------------- TMUX & Misc ----------------------- |
  std::vector<int> selected_tmux_window_;
  std::string      session_name_;
  const int        MAX_SELECTED_TMUX_WINDOWS = 2;
  int              terminal_cols_ = 0, terminal_lines_ = 0;


  void prefillUavStatus();
  void setupDisplayText();

  long         last_gigas_                = 0;
  bool         have_data_                 = false;
  bool         have_short_data_           = false;
  int          estimator_display_counter_ = 0;
  bool         increment_counter_         = false;
  rclcpp::Time last_time_got_data_;
  rclcpp::Time last_time_got_short_data_;
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
  WindowPtr node_stats_window_;
  WindowPtr general_info_window_;
  WindowPtr debug_window_;
  WindowPtr sub_tmux_window_1_;
  WindowPtr sub_tmux_window_2_;
  WindowPtr string_window_;

  // | ----------------------- Data Storage --------------------- |
  std::vector<tui::StatusWindow> menu_vec_;
  std::vector<tui::StatusWindow> submenu_vec_;
  std::vector<tui::ControlBar>   goto_menu_inputs_;

  std::vector<TopicInfo>   string_topic_;
  std::vector<Service>     service_vec_;
  std::vector<std::string> service_input_vec_;
  std::vector<std::string> main_menu_text_;
  std::vector<std::string> display_menu_text_;
  std::vector<std::string> goto_menu_text_;
  std::vector<double>      goto_double_vec_;
};

} // namespace mrs_uav_status::tui
