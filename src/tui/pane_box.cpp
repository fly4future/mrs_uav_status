/* includes //{ */

#include <mrs_uav_status/tui/pane_box.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/utils/helpers.hpp>

//}

namespace mrs_uav_status::tui
{

/* PaneBox() //{ */

PaneBox::PaneBox() {
  // Default pane: available sensors (dynamic, plugin-driven) — name/rate/status
  panes_.push_back({"Sensors", [this](WINDOW *win) { renderSensorsPane(win); }, nullptr});
  // ROS per-node CPU usage (was its own top-right box)
  panes_.push_back({"ROS Node CPU", [this](WINDOW *win) { renderNodeCpuPane(win); }, nullptr});
  // GNSS fix + custom display strings
  panes_.push_back({"GNSS & strings", [this](WINDOW *win) { renderStringsGnssPane(win); }, nullptr});
  // Problems + errors. Auto-focused when either becomes non-empty
  panes_.push_back({"Problems & errors", [this](WINDOW *win) { renderProblemsPane(win); },
                    [this]() {
                      // Avoid auto-focusing on frozen stale problems/errors.
                      return snapshot_->freshness.general_robot_info &&
                             (!snapshot_->general_robot_info.problems_preventing_start.empty() || !snapshot_->general_robot_info.errors.empty());
                    }});

  pane_focus_prev_.assign(panes_.size(), false);
  pane_idx_ = 0;
}

//}

/* cycle() //{ */

void PaneBox::cycle() {
  if (panes_.empty()) {
    return;
  }
  pane_idx_ = (pane_idx_ + 1) % panes_.size();
}

//}

/* select() //{ */

void PaneBox::select(std::size_t idx) {
  if (idx < panes_.size()) {
    pane_idx_ = idx;
  }
}

//}

/* render() //{ */

void PaneBox::render(WINDOW *win, const status::RenderSnapshot &snapshot, bool light_scheme, bool mini) {
  if (!win || panes_.empty()) {
    return;
  }

  snapshot_     = &snapshot;
  light_scheme_ = light_scheme;
  mini_         = mini;

  // auto-focus: when a pane wants_focus switching to it
  for (std::size_t i = 0; i < panes_.size(); ++i) {
    const bool wants = panes_[i].wants_focus && panes_[i].wants_focus();
    if (wants && !pane_focus_prev_[i]) {
      pane_idx_ = i;
    }
    pane_focus_prev_[i] = wants;
  }

  panes_[pane_idx_].render(win);

  snapshot_ = nullptr;
}

//}

/* drawPaneChrome() //{ */

int PaneBox::drawPaneChrome(WINDOW *win) {
  const status::BorderStatus bs = snapshot_->border_status;

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, bs.avoiding_collision, bs.bumper_active, bs.can_takeoff, bs.null_tracker);

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }

  // Tab bar in the top border (row 0): numbered tabs. The active tab shows its
  // number + name in brackets (green); the others show just their number in red
  // to signal they're switchable.
  int x = 2;
  for (std::size_t i = 0; i < panes_.size(); ++i) {
    const bool        active = (i == pane_idx_);
    const std::string num    = std::to_string(i + 1);

    if (active) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      const std::string tab = num + " [" + panes_[i].title + "]";
      mvwaddstr(win, 0, x, tab.c_str());
      x += static_cast<int>(tab.size());
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
    } else {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwaddstr(win, 0, x, num.c_str());
      x += static_cast<int>(num.size());
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }
    x += 1; // single-space gap between tabs
  }

  return 1;
}

//}

/* renderStringsGnssPane() //{ */

void PaneBox::renderStringsGnssPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<std::string> string_vector;
  uint8_t                  gnss_fix_type      = 0;
  uint8_t                  gnss_num_sats      = 0;
  double                   gnss_pos_acc       = 100.0;
  double                   gnss_status_rate   = 0.0;
  bool                     gnss_status_msg_ok = false;
  bool                     have_system_health_info;

  {
    have_system_health_info = snapshot_->freshness.system_health_info;
    if (const auto *gnss = utils::findSensor(snapshot_->system_health_info.available_sensors, status::SENSOR_TYPE_GNSS); gnss) {
      const std::string fix_type_raw = utils::lookupDetail(gnss->details, "fix_type");
      gnss_status_msg_ok             = (fix_type_raw != "nan");
      gnss_fix_type                  = static_cast<uint8_t>(utils::parseLongOr(fix_type_raw, 0));
      gnss_num_sats                  = static_cast<uint8_t>(utils::parseLongOr(utils::lookupDetail(gnss->details, "num_satellites"), 0));
      gnss_pos_acc                   = utils::parseDoubleOr(utils::lookupDetail(gnss->details, "position_accuracy"), 100.0);
      gnss_status_rate               = gnss->rate;
    }

    // Eviction already happened in UavStatusCore::pruneStrings() (top of every tick());
    // snapshot_->display_strings is whatever's currently live for display.
    for (const auto &entry : snapshot_->display_strings) {
      string_vector.push_back(entry);
    }
  }

  // gnss_status_rate/gnss_status_msg_ok are cached and never reset, so gate on freshness too.
  if (have_system_health_info && gnss_status_rate > 0.0 && gnss_status_msg_ok) {
    std::string fix_string;
    if (gnss_fix_type < 1 || gnss_fix_type >= 8) {
      fix_string += "-r ";
    }
    fix_string += "Fix Type: ";

    switch (gnss_fix_type) {
    case 0:
      fix_string += "NO GNSS";
      break;
    case 1:
      fix_string += "NO FIX";
      break;
    case 2:
      fix_string += "2D FIX";
      break;
    case 3:
      fix_string += "3D FIX";
      break;
    case 4:
      fix_string += "3D SBAS FIX";
      break;
    case 5:
      fix_string += "RTK FLOAT";
      break;
    case 6:
      fix_string += "RTK FIX (INT)";
      break;
    case 7:
      fix_string += "STATIC - BASESTATION";
      break;
    case 8:
      fix_string += "PPP 3D FIX";
      break;
    default:
      fix_string += "UNKNOWN";
      break;
    }

    std::string gnss_acc_string;
    if (gnss_pos_acc >= 100.0) {
      gnss_acc_string = "N/A";
    } else {
      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << gnss_pos_acc;
      gnss_acc_string = stream.str();
    }
    string_vector.push_back(fix_string);
    string_vector.push_back("Num sats: " + std::to_string(gnss_num_sats) + " Acc: " + gnss_acc_string + " m");
  }

  if (string_vector.empty()) {
    string_vector.push_back("-y no GNSS / strings data");
  }

  constexpr int MAX_STRING_ROWS = 9;
  for (const auto &raw : string_vector) {
    if (row > MAX_STRING_ROWS) {
      break;
    }

    int         tmp_color = static_cast<int>(ColorPair::Normal);
    bool        blink     = false;
    std::string display   = raw;

    // In-band colour tag: leading "-R"/"-r"/"-Y"/"-y"/"-G"/"-g" (uppercase blinks).
    if (display.size() >= 3 && display[0] == '-') {
      switch (display[1]) {
      case 'R':
        blink = true;
        [[fallthrough]];
      case 'r':
        tmp_color = static_cast<int>(ColorPair::Red);
        break;
      case 'Y':
        blink = true;
        [[fallthrough]];
      case 'y':
        tmp_color = static_cast<int>(ColorPair::Yellow);
        break;
      case 'G':
        blink = true;
        [[fallthrough]];
      case 'g':
        tmp_color = static_cast<int>(ColorPair::Green);
        break;
      }
      if (tmp_color != static_cast<int>(ColorPair::Normal)) {
        display.erase(0, 3);
      }
    }

    if (blink) {
      wattron(win, A_BLINK);
    }
    wattron(win, COLOR_PAIR(tmp_color));
    row = printWrappedString(win, row, 1, display, 80, MAX_STRING_ROWS);
    wattroff(win, COLOR_PAIR(tmp_color));
    wattroff(win, A_BLINK);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* renderProblemsPane() //{ */

void PaneBox::renderProblemsPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  const std::vector<std::string> &problems                = snapshot_->general_robot_info.problems_preventing_start;
  const std::vector<std::string> &errors                  = snapshot_->general_robot_info.errors;
  const bool                      have_general_robot_info = snapshot_->freshness.general_robot_info;

  // Empty vectors are indistinguishable from "genuinely zero problems" without this check.
  if (!have_general_robot_info) {
    printNoData(win, row, 1, false);
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  constexpr int MAX_PROBLEM_ROWS = 9;
  constexpr int TEXT_WIDTH       = 80;

  // Errors render first: they matter even mid-flight.
  const auto errors_color = errors.empty() ? ColorPair::Green : ColorPair::Red;
  wattron(win, COLOR_PAIR(static_cast<int>(errors_color)));
  printLimitedString(win, row++, 1, "Errors: " + std::to_string(errors.size()), TEXT_WIDTH);
  wattroff(win, COLOR_PAIR(static_cast<int>(errors_color)));

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  for (const auto &e : errors) {
    if (row > MAX_PROBLEM_ROWS) {
      break;
    }
    row = printWrappedString(win, row, 1, "- " + e, TEXT_WIDTH, MAX_PROBLEM_ROWS);
  }
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  // Problems preventing start no longer apply once a real tracker (not NullTracker) is active.
  const bool is_flying = snapshot_->freshness.control_info && !snapshot_->border_status.null_tracker;

  if (!is_flying) {
    if (row <= MAX_PROBLEM_ROWS) {
      ++row; // blank separator
    }

    if (row <= MAX_PROBLEM_ROWS) {
      const auto problems_color = problems.empty() ? ColorPair::Green : ColorPair::Red;
      wattron(win, COLOR_PAIR(static_cast<int>(problems_color)));
      printLimitedString(win, row++, 1, "Problems preventing start: " + std::to_string(problems.size()), TEXT_WIDTH);
      wattroff(win, COLOR_PAIR(static_cast<int>(problems_color)));
    }

    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    for (const auto &p : problems) {
      if (row > MAX_PROBLEM_ROWS) {
        break;
      }
      row = printWrappedString(win, row, 1, "- " + p, TEXT_WIDTH, MAX_PROBLEM_ROWS);
    }
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* renderSensorsPane() //{ */

void PaneBox::renderSensorsPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<status::SensorStatusData> sensors                 = snapshot_->system_health_info.available_sensors;
  const bool                            have_system_health_info = snapshot_->freshness.system_health_info;

  if (!have_system_health_info) {
    // Otherwise the last real sensor list/rates/statuses render forever, looking healthy.
    printNoData(win, row, 1, mini_);
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  // Column header.
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 1, "sensor", 36);
  printLimitedString(win, row, 42, "rate", 8);
  printLimitedString(win, row, 55, "status", 12);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  ++row;

  // Worst-first: ERROR, WARN, STALE, then OK -- so the sensors most worth seeing
  // survive the MAX_SENSOR_ROWS cutoff below when the list is longer than it.
  auto severity_rank = [](uint8_t level) -> int {
    switch (level) {
    case status::SENSOR_STATUS_ERROR:
      return 0;
    case status::SENSOR_STATUS_WARN:
      return 1;
    case status::SENSOR_STATUS_STALE:
      return 2;
    default: // OK
      return 3;
    }
  };
  std::stable_sort(sensors.begin(), sensors.end(), [&severity_rank](const auto &a, const auto &b) { return severity_rank(a.level) < severity_rank(b.level); });

  constexpr int MAX_SENSOR_ROWS = 9;
  constexpr int MESSAGE_WIDTH   = 80;
  for (const auto &s : sensors) {
    if (row > MAX_SENSOR_ROWS) {
      break;
    }

    printLimitedString(win, row, 1, s.name, 36);
    printLimitedDouble(win, row, 40, "%6.1f", s.rate, 100000);

    // SensorStatus.level: 0=OK, 1=WARN, 2=ERROR, 3=STALE.
    int         color = static_cast<int>(ColorPair::Normal);
    std::string label = "?";
    switch (s.level) {
    case 0:
      color = static_cast<int>(ColorPair::Green);
      label = "OK";
      break;
    case 1:
      color = static_cast<int>(ColorPair::Yellow);
      label = "WARN";
      break;
    case 2:
      color = static_cast<int>(ColorPair::Red);
      label = "ERROR";
      break;
    case 3:
      color = static_cast<int>(ColorPair::Yellow);
      label = "STALE";
      break;
    default:
      break;
    }
    wattron(win, COLOR_PAIR(color));
    printLimitedString(win, row, 55, label, 12);
    ++row;

    if (s.level != status::SENSOR_STATUS_OK && row <= MAX_SENSOR_ROWS) {
      row = printWrappedString(win, row, 1, "    -> " + s.message, MESSAGE_WIDTH, MAX_SENSOR_ROWS);
    }
    wattroff(win, COLOR_PAIR(color));
  }

  if (sensors.empty()) {
    printLimitedString(win, row, 1, "no sensors reported", 40);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* renderNodeCpuPane() //{ */

void PaneBox::renderNodeCpuPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<status::CpuLoadData> node_cpu_loads          = snapshot_->system_health_info.onboard_computer_info.node_cpu_loads;
  const bool                       have_system_health_info = snapshot_->freshness.system_health_info;

  if (!have_system_health_info) {
    // Otherwise an empty node_cpu_loads sums to 0.0, rendering as a healthy-looking 0% CPU.
    printNoData(win, row, 1, mini_);
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  // Aggregate (single-core %) total — OnboardComputerInfo doesn't expose it.
  double cpu_load_total = 0.0;
  for (const auto &n : node_cpu_loads) {
    cpu_load_total += n.cpu_load;
  }

  constexpr int MAX_NODE_ROWS = 9;

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 42, "Total CPU", 20);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedDouble(win, row, 55, "%5.1f", cpu_load_total, 9999);
  printLimitedString(win, row, 61, "%", 5);
  ++row;

  mvwhline(win, row, 55, ACS_HLINE, 7);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 1, "node", 36);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  ++row;

  // Sort by CPU load descending, then name ascending for tie-breaking.
  std::sort(node_cpu_loads.begin(), node_cpu_loads.end(),
            [](const auto &a, const auto &b) { return (a.cpu_load > b.cpu_load) || ((a.cpu_load == b.cpu_load) && (a.node_name < b.node_name)); });

  for (const auto &n : node_cpu_loads) {
    if (row > MAX_NODE_ROWS) {
      break;
    }
    printLimitedString(win, row, 1, n.node_name, 36);

    short tmp_color = static_cast<int>(ColorPair::Green);
    if (n.cpu_load > 99.9) {
      tmp_color = static_cast<int>(ColorPair::Red);
    } else if (n.cpu_load > 49.9) {
      tmp_color = static_cast<int>(ColorPair::Yellow);
    }
    wattron(win, COLOR_PAIR(tmp_color));
    printLimitedDouble(win, row, 55, "%5.1f", n.cpu_load, 9999);
    wattroff(win, COLOR_PAIR(tmp_color));
    ++row;
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

} // namespace mrs_uav_status::tui
