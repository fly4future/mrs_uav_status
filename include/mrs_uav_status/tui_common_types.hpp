#pragma once

#include <ncurses.h>

#include <vector>
#include <string>

namespace mrs_uav_status
{
namespace tui
{

class InputBox {
public:
  InputBox(int size, WINDOW *win, double initial_value);
  unsigned long process(int key_in);
  void          print(int line, bool active);
  double        getDouble();

  inline static unsigned long cursor_;

private:
  WINDOW           *win_;
  unsigned long     size_;
  std::vector<char> buffer_;
};

class Menu {
public:
  Menu(int begin_y, int begin_x, std::vector<std::string> &text);
  Menu(int begin_y, int begin_x, std::vector<std::string> &text, int id);

  WINDOW *getWin();
  int     getLine();
  int     getId();

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

  Result iterate(std::vector<std::string> &text, int key, bool refresh);
  Result iterate(int key, bool refresh);

private:
  WINDOW                  *win_;
  int                      line_ = 0;
  int                      id_;
  int                      y_;
  int                      x_;
  int                      rows_;
  int                      cols_;
  std::vector<std::string> text_;
};

} // namespace tui
} // namespace mrs_uav_status
