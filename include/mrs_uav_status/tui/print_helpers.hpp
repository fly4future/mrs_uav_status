#pragma once

#include <ncurses.h>
#include <string>
#include <algorithm>
#include <cmath>
#include <vector>

#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/utils/terminal.hpp>

namespace mrs_uav_status::tui
{

inline void printLimitedInt(WINDOW *win, int y, int x, const std::string &str_in, int num, int limit) {
  std::string str_out = str_in;
  if (std::abs(num) > limit) {
    for (unsigned long i = 0; i < str_out.length() - 2; i++) {
      if (str_out[i] == '.' && str_out[i + 2] == 'i') {
        str_out[i + 1] = '0';
        str_out[i + 2] = 'e';
        break;
      }
    }
  }
  mvwprintw(win, y, x, str_out.c_str(), num);
}

inline void printLimitedDouble(WINDOW *win, int y, int x, const std::string &str_in, double num, double limit) {
  std::string format_str = str_in;
  if (std::abs(num) > limit) {
    for (unsigned long i = 0; i < format_str.length() - 2; i++) {
      if (format_str[i] == '.' && format_str[i + 2] == 'f') {
        format_str[i + 1] = '0';
        format_str[i + 2] = 'e';
        break;
      }
    }
  }
  mvwprintw(win, y, x, format_str.c_str(), num);
}

inline void printLimitedString(WINDOW *win, int y, int x, const std::string &str_in, unsigned long limit) {
  if (str_in.length() > limit) {
    std::string truncated_str = str_in.substr(0, limit);
    mvwprintw(win, y, x, "%s", truncated_str.c_str());
  } else {
    mvwprintw(win, y, x, "%s", str_in.c_str());
  }
}

inline void printCompressedLimitedString(WINDOW *win, int y, int x, const std::string &str_in, unsigned long limit) {
  if (str_in.empty()) {
    return;
  }
  std::string compressed = str_in;
  std::string chars_to_remove("aeiouAEIOU :");
  for (char c : chars_to_remove) {
    if (compressed.length() > 1) {
      compressed.erase(std::remove(compressed.begin() + 1, compressed.end(), c), compressed.end());
    }
  }
  if (compressed.length() > limit) {
    compressed.resize(limit);
  }
  mvwprintw(win, y, x, "%s", compressed.c_str());
}

inline void printNoData(WINDOW *win, int y, int x, [[maybe_unused]] bool mini) {
  wattron(win, A_BLINK);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  mvwprintw(win, y, x, "NO DATA");
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  wattroff(win, A_BLINK);
}

inline void printNoData(WINDOW *win, int y, int x, const std::string &text, bool mini) {
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  mvwprintw(win, y, x, text.c_str());
  printNoData(win, y, x + static_cast<int>(text.length()), mini);
}

inline void printBox(WINDOW *win, bool avoiding_collision, bool bumper_active, bool can_takeoff, bool null_tracker) {
  if (avoiding_collision || bumper_active) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    wattron(win, A_BLINK);
    wattron(win, A_STANDOUT);
  }
  if (!can_takeoff && null_tracker) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
    wattron(win, A_STANDOUT);
  }
  box(win, 0, 0);
  wattroff(win, A_BLINK);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  wattroff(win, A_STANDOUT);
}

inline void printError(WINDOW *win, const std::string &msg) {
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  printLimitedString(win, 0, 0, msg, 120);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  wnoutrefresh(win);
}

inline void printDebug(WINDOW *win, const std::string &msg) {
  printLimitedString(win, 0, 0, msg, 120);
  wnoutrefresh(win);
}

inline void printServiceResult(WINDOW *win, bool light, bool success, const std::string &msg) {
  if (light) {
    wattron(win, A_STANDOUT);
  }
  werase(win);
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  if (success) {
    printLimitedString(win, 0, 0, "Service call success: " + msg, 120);
  } else {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    printLimitedString(win, 0, 0, "Service call failed: " + msg, 120);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  }
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  wattroff(win, A_BOLD);
}

inline void printHelp(WINDOW *win, bool help_active) {
  werase(win);
  if (help_active) {
    printLimitedString(win, 1, 0, "How to use mrs_status:", 120);
    printLimitedString(win, 2, 0, "Press the 'm' key to enter a services menu", 120);
    printLimitedString(win, 3, 0, "Press the 'g' key to set a goto reference", 120);
    printLimitedString(win, 4, 0, "Press the 'M' key to switch into minimalistic mode, which takes less screen space", 120);
    printLimitedString(win, 5, 0, "Press the 'R' key to enter 'remote' mode to take direct control of the uav with your keyboad", 120);
    printLimitedString(win, 6, 0, "   In remote mode, use these keys to control the drone:", 120);
    printLimitedString(win, 7, 0, "      'w','s','a','d' to control pitch and roll ('h','j','k','l' works too)", 120);
    printLimitedString(win, 8, 0, "      'q','e'         to control heading", 120);
    printLimitedString(win, 9, 0, "      'r','f'         to control altitude", 120);
    printLimitedString(win, 10, 0, "      'G'             to switch controlling in the FCU frame (local) or the world frame (global)", 120);
    printLimitedString(win, 12, 0, "You can also display any info from your node in the mrs_status:", 120);
    printLimitedString(win, 14, 0, "   topic: mrs_status/display_string (std_msgs::String)", 120);
    printLimitedString(win, 15, 0, "   - Publish any string to this topic and it will show up in mrs_status", 120);
    printLimitedString(win, 17, 0, "Press 'D' to display info from other panes of this tmux session, up to 2 panes can be viewed", 120);
    printLimitedString(win, 18, 0,
                       "Press 'p' to cycle the top-right pane through various system info (problems, node CPU usage, GPS status, available sensors)", 120);
    printLimitedString(win, 19, 0, "Press '1'-'9' to directly select a pane ", 120);

    printLimitedString(win, 21, 0, "Press 'h' to hide help", 120);
  } else {
    printLimitedString(win, 1, 0, "Press 'h' key for help", 120);
  }
  wnoutrefresh(win);
}

inline void printTmuxDump(WINDOW *debug_window, WINDOW *sub1, WINDOW *sub2, const std::vector<int> &selected, const std::string &session_name,
                          const std::vector<std::string> &display_menu_text, int max_windows, bool avoiding_collision, bool bumper_active, bool can_takeoff,
                          bool null_tracker) {
  werase(debug_window);
  printBox(debug_window, avoiding_collision, bumper_active, can_takeoff, null_tracker);

  if (static_cast<int>(selected.size()) > max_windows) {
    return;
  }

  int tmp_cols, tmp_rows;
  getmaxyx(sub1, tmp_rows, tmp_cols);

  for (size_t i = 0; i < selected.size(); i++) {
    std::string command_str = "tmux resize-window -t " + session_name + ":" + std::to_string(selected[i]) + " -A";
    mrs_uav_status::utils::callTerminal(command_str.c_str());
    command_str          = "tmux capture-pane -pt " + session_name + ":" + std::to_string(selected[i]) + " -S 0 | tail -n " + std::to_string(tmp_rows + 1);
    std::string response = mrs_uav_status::utils::callTerminal(command_str.c_str());
    switch (i) {
    case 0:
      mvwaddstr(sub1, 0, 0, response.c_str());
      break;
    case 1:
      mvwaddstr(sub2, 0, 0, response.c_str());
      break;
    }
  }

  mvwhline(debug_window, tmp_rows + 1, 1, 0, tmp_cols - 1);

  // A persisted index may no longer be valid if the tmux window count changed since it was saved.
  if (selected.size() > 1 && selected[1] >= 0 && static_cast<size_t>(selected[1]) < display_menu_text.size()) {
    printLimitedString(debug_window, tmp_rows + 1, 3, display_menu_text[selected[1]], 50);
  }
  if (!selected.empty() && selected[0] >= 0 && static_cast<size_t>(selected[0]) < display_menu_text.size()) {
    printLimitedString(debug_window, 0, 3, display_menu_text[selected[0]], 50);
  }

  wnoutrefresh(debug_window);
  touchwin(debug_window);
  wnoutrefresh(sub1);
  wnoutrefresh(sub2);
}

// Map a measured Hz against an expected Hz to a ColorPair (Green ≥ 90% expected,
// Yellow ≥ 50% expected, Red otherwise). Replaces the precomputed *_color fields
// that the legacy UavStatus blob carried.
inline int16_t rateColor(double rate, double expected) {
  if (expected <= 0.0) {
    return static_cast<int16_t>(ColorPair::Normal);
  }
  if (rate >= 0.9 * expected) {
    return static_cast<int16_t>(ColorPair::Green);
  }
  if (rate >= 0.5 * expected) {
    return static_cast<int16_t>(ColorPair::Yellow);
  }
  return static_cast<int16_t>(ColorPair::Red);
}

// btop-style hotkey hint: render @p word with its first character (the trigger
// key) in red+bold and the remainder in the normal colour. Returns the column
// just past the word (plus one space) so hints can be chained left-to-right.
inline int printHotkey(WINDOW *win, int y, int x, const std::string &word) {
  if (word.empty()) {
    return x;
  }
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  mvwaddch(win, y, x, static_cast<chtype>(word.front()));
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  mvwaddstr(win, y, x + 1, word.substr(1).c_str());
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  wattroff(win, A_BOLD);

  return x + static_cast<int>(word.size()) + 1; // +1 for a single-space gap
}

} // namespace mrs_uav_status::tui
