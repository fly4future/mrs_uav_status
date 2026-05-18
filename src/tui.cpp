#include <mrs_uav_status/tui/tui.hpp>

namespace mrs_uav_status::tui
{

TUI::TUI(const std::string &colorscheme, bool colorblind_mode, bool minimized_mode)
    : _colorscheme_(colorscheme), _colorblind_mode_(colorblind_mode), _minimized_mode_(minimized_mode) {
  // initscr();
  // cbreak();
  // noecho();
  // curs_set(0);
  // nodelay(stdscr, TRUE);
  // keypad(stdscr, TRUE);
}

bool TUI::updateTermSize() {

  bool changed = false;

  std::string command  = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string response = utils::callTerminal(command.c_str());

  std::vector<std::string> results;

  results = mrs_uav_status::utils::splitByChar(response, 'x');

  int cols, lines;

  try {
    cols  = std::stoi(results[0]);
    lines = std::stoi(results[1]);
  }

  catch (const std::invalid_argument &e) {
    cols  = 0;
    lines = 0;
  }

  if (terminal_cols_ != cols || terminal_lines_ != lines) {
    terminal_lines_  = lines;
    terminal_cols_   = cols;
    changed = true;
  }

  return (changed);
}

void TUI::setupWindows(bool have_data) {

  std::string command = "tmux display-message -p '#S'";
  session_name_       = utils::callTerminal(command.c_str());
  session_name_.erase(std::remove(session_name_.begin(), session_name_.end(), '\n'), session_name_.end());

  command                           = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string              response = utils::callTerminal(command.c_str());
  std::vector<std::string> results;
  results = mrs_uav_status::utils::splitByChar(response, 'x');

  if (_minimized_mode_) {

    control_manager_window_ = newwin(4, 9, 1, 1);
    uav_state_window_       = newwin(6, 9, 5, 1);
    top_bar_window_         = newwin(1, 140, 0, 1);
    general_info_window_    = newwin(4, 9, 1, 10);
    hw_api_state_window_    = newwin(6, 9, 5, 10);
    debug_window_           = newwin(terminal_lines_ - 15, terminal_cols_ - 1, 13, 1);
    generic_topic_window_   = newwin(10, 9, 1, 19);
    string_window_          = newwin(10, 15, 1, 28);
    bottom_window_          = newwin(1, 120, 11, 1);

  } else {

    uav_state_window_       = newwin(7, 26, 5, 1);
    control_manager_window_ = newwin(4, 26, 1, 1);
    hw_api_state_window_    = newwin(7, 25, 5, 27);
    general_info_window_    = newwin(4, 25, 1, 27);
    top_bar_window_         = newwin(1, 140, 0, 1);
    bottom_window_          = newwin(1, 120, 12, 1);
    debug_window_           = newwin(terminal_lines_ - 15, terminal_cols_ - 1, 13, 1);
    int half_lines          = (terminal_lines_ - 18) / 2;
    sub_tmux_window_1_      = derwin(debug_window_, half_lines, terminal_cols_ - 3, 1, 1);
    sub_tmux_window_2_      = derwin(debug_window_, half_lines, terminal_cols_ - 3, half_lines + 2, 1);

    generic_topic_window_ = newwin(11, 25, 1, 52);
    string_window_        = newwin(11, 32, 1, 77);
    node_stats_window_    = newwin(11, 50, 1, 109);
  }

  clear();
  _light_ = tui::setupColors(have_data, _colorscheme_, _colorblind_mode_);
}


} // namespace mrs_uav_status::tui
