#pragma once
#include <string>
#include <vector>
#include <functional>

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
#include <mrs_msgs/msg/reference.hpp>

namespace mrs_uav_status::tui
{
class TUI {
public:
  TUI(const std::string &colorscheme, bool colorblind_mode, bool minimized_mode);

  // | --------------------- Window Handlers -------------------- |
  void setupWindows(bool have_data);
  void stringHandler(WINDOW *win);
  void uavStateHandler(WINDOW *win);
  void nodeStatsHandler(WINDOW *win);
  void hwApiStateHandler(WINDOW *win);
  void generalInfoHandler(WINDOW *win);
  void genericTopicHandler(WINDOW *win);
  void controlManagerHandler(WINDOW *win);

private:
  std::string _colorscheme_;
  bool        _colorblind_mode_;
  bool        _minimized_mode_;
  bool        _light_ = false;

  struct MenuRow
  {
    std::string           label;
    std::function<void()> on_open;
  };

  std::vector<MenuRow> main_menu_rows_;
  std::vector<MenuRow> sub_menu_rows_;

  // | -------------------- Remote & Flight --------------------- |
  void remoteHandler(int key, WINDOW *win);
  void gimbalHandler(int key, WINDOW *win);
  void remoteModeFly(const mrs_msgs::msg::Reference &ref_in);

  // | ------------------- Menu & Input Logic ------------------- |
  void        setupMainMenu();
  void        setupGotoMenu();
  void        setupDisplayMenu();
  void        setupDisplayText();
  bool        mainMenuHandler(int key_in);
  bool        gotoMenuHandler(int key_in);
  bool        displayMenuHandler(int key_in);
  static bool isValidMenuIndex(int index, size_t container_size);
  void        createSubMenu(std::vector<std::string> &submenu_entries);
  void        createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<mrs_msgs::srv::String> &service_client);
  void        createSubMenuActions(std::vector<std::string> &submenu_entries, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> &service_client);

  // | ---------------------- TMUX & Misc ----------------------- |
  std::vector<int> selected_tmux_window_;
  std::string      session_name_;
  const int        MAX_SELECTED_TMUX_WINDOWS = 2;
  int              terminal_cols_ = 0, terminal_lines_ = 0;


  bool updateTermSize();
  void prefillUavStatus();
  void topLineHandler(WINDOW *win);


  // | ---------------------- Window Pointers ------------------- |
  WINDOW *uav_state_window_       = nullptr;
  WINDOW *control_manager_window_ = nullptr;
  WINDOW *hw_api_state_window_    = nullptr;
  WINDOW *top_bar_window_         = nullptr;
  WINDOW *bottom_window_          = nullptr;
  WINDOW *generic_topic_window_   = nullptr;
  WINDOW *node_stats_window_      = nullptr;
  WINDOW *general_info_window_    = nullptr;
  WINDOW *debug_window_           = nullptr;
  WINDOW *sub_tmux_window_1_      = nullptr;
  WINDOW *sub_tmux_window_2_      = nullptr;
  WINDOW *string_window_          = nullptr;

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
