#pragma once

#include <ncurses.h>

#include <vector>
#include <string>

namespace mrs_uav_status::tui
{

class StatusWindow {
public:
  StatusWindow(int begin_y, int begin_x, const std::vector<std::string> &text);

  WINDOW *getWin() const;
  int     getLine() const;

  struct Result
  {
    enum class Action
    {
      None,
      Select,
      Exit
    } action = Action::None;

    int selected_line = -1;
    int pressed_key   = -1;
  };

  Result iterate(const std::vector<std::string> &text, int key, bool refresh);
  Result iterate(int key, bool refresh);

private:
  WINDOW                  *win_  = nullptr;
  int                      line_ = 0;
  std::vector<std::string> text_;
};

} // namespace mrs_uav_status::tui
