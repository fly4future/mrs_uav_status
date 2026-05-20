#pragma once

#include <ncurses.h>

#include <vector>

namespace mrs_uav_status::tui
{

class ControlBar {
public:
  ControlBar(unsigned long size, WINDOW *win, double initial_value);
  
  unsigned long process(int key_in);
  void          print(int line, bool active);
  
  double        getDouble() const;

  inline static unsigned long cursor_ = 0;

private:
  WINDOW            *win_;
  unsigned long      size_;
  std::vector<char> buffer_;
};

} // namespace mrs_uav_status
