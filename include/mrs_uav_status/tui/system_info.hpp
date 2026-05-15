#pragma once

#include <ncurses.h>

#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status
{
namespace tui
{

inline void printCpuLoad(WINDOW *win, double cpu_load, bool mini) {
  int tmp_color = static_cast<int>(ColorPair::Green);
  if (cpu_load > 80.0) {
    tmp_color = static_cast<int>(ColorPair::Red);
  } else if (cpu_load > 60.0) {
    tmp_color = static_cast<int>(ColorPair::Yellow);
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini) {
    printLimitedString(win, 1, 1, "CPU", 3);
  } else {
    printLimitedDouble(win, 1, 1, "CPU: %4.1f %%", cpu_load, 99.9);
  }
}

inline void printCpuTemp(WINDOW *win, double cpu_temp, bool mini) {
  int tmp_color = static_cast<int>(ColorPair::Green);
  if (cpu_temp > 90.0) {
    tmp_color = static_cast<int>(ColorPair::Red);
  } else if (cpu_temp > 75.0) {
    tmp_color = static_cast<int>(ColorPair::Yellow);
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini) {
    printLimitedDouble(win, 0, 1, "%3.0f °C", cpu_temp, 999.9);
  } else {
    printLimitedDouble(win, 0, 1, "%5.1f °C", cpu_temp, 999.9);
  }
}

inline void printCpuFreq(WINDOW *win, double cpu_ghz) {
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedDouble(win, 1, 16, "%4.2f GHz", cpu_ghz, 10);
}

inline void printMemLoad(WINDOW *win, double free_ram, double total_ram, bool mini) {
  double used_ram  = total_ram - free_ram;
  double ram_ratio = used_ram / total_ram;

  int tmp_color = static_cast<int>(ColorPair::Green);
  if (ram_ratio > 0.7) {
    tmp_color = static_cast<int>(ColorPair::Red);
    wattron(win, A_BLINK);
  } else if (ram_ratio > 0.5) {
    tmp_color = static_cast<int>(ColorPair::Yellow);
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini) {
    printLimitedString(win, 2, 1, "RAM", 3);
  } else {
    printLimitedDouble(win, 2, 1, "RAM: %4.1f G", free_ram, 100);
  }
  wattroff(win, A_BLINK);
}

inline void printDiskSpace(WINDOW *win, int free_hdd, long last_gigas, bool mini) {
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

  if (free_hdd < 20 || free_hdd != last_gigas) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
  }

  if (free_hdd < 10) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    if (mini) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedDouble(win, 2, 5, "%3.1f", double(free_hdd), 10);
    } else {
      printLimitedDouble(win, 2, 14, "HDD: %3.1f G", double(free_hdd), 10);
    }
  }

  if (free_hdd < 100) {
    if (mini) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 6, "%i", free_hdd, 1000);
    } else {
      printLimitedInt(win, 2, 14, "HDD:  %i G", free_hdd, 1000);
    }
  }

  if (free_hdd < 1024) {
    if (mini) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 5, "%i", free_hdd, 1000);
    } else {
      printLimitedInt(win, 2, 14, "HDD: %i G", free_hdd, 1000);
    }
  } else {
    if (mini) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 5, "%i T", free_hdd / 1024, 10);
    } else {
      printLimitedDouble(win, 2, 14, "HDD: %3.1f T", free_hdd / 1024.0, 1000);
    }
  }
}

} // namespace tui
} // namespace mrs_uav_status
