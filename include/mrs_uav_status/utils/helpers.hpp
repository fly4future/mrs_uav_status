#pragma once

#include <cstdint>
#include <exception>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/sensor_status.hpp>

namespace mrs_uav_status::utils
{

inline std::vector<std::string> splitByChar(const std::string &input, char delimiter) {

  if (input.empty()) {
    return {""};
  }

  std::vector<std::string> result;
  std::size_t              start = 0;

  while (start < input.size()) {
    const std::size_t pos = input.find(delimiter, start);
    if (pos == std::string::npos) {
      result.emplace_back(input.substr(start));
      break;
    }
    result.emplace_back(input.substr(start, pos - start));
    start = pos + 1;
  }

  return result;
}

// Returns {active, available\active} — the legacy UavStatus convention is
// "first element is the active one, rest are alternates."
inline std::vector<std::string> withActiveFirst(const std::string &active, const std::vector<std::string> &available) {
  std::vector<std::string> out;
  out.reserve(available.size() + 1);
  if (!active.empty()) {
    out.push_back(active);
  }
  for (const auto &s : available) {
    if (s != active) {
      out.push_back(s);
    }
  }
  return out;
}

// No UNKNOWN sentinel in the enum (0 == DRONE) — caller must check message freshness separately.
inline std::string robotTypeToString(uint8_t robot_type) {
  switch (robot_type) {
    case mrs_msgs::msg::GeneralRobotInfo::ROBOT_TYPE_DRONE:
      return "DRONE";
    case mrs_msgs::msg::GeneralRobotInfo::ROBOT_TYPE_BOAT:
      return "BOAT";
    case mrs_msgs::msg::GeneralRobotInfo::ROBOT_TYPE_GROUND_ROBOT:
      return "UGV";
    default:
      return "UNKNOWN";
  }
}

// Find the first SensorStatus of a given type. Returns nullptr if not present.
inline const mrs_msgs::msg::SensorStatus *findSensor(const std::vector<mrs_msgs::msg::SensorStatus> &sensors, uint8_t type) {
  for (const auto &s : sensors) {
    if (s.type == type) {
      return &s;
    }
  }
  return nullptr;
}

// Look up a value in a KeyValue list. Returns fallback if the key isn't present.
inline std::string lookupDetail(const std::vector<diagnostic_msgs::msg::KeyValue> &details, const std::string &key, const std::string &fallback = "") {
  for (const auto &kv : details) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return fallback;
}

inline double parseDoubleOr(const std::string &s, double fallback) {
  if (s.empty()) {
    return fallback;
  }
  try {
    return std::stod(s);
  }
  catch (const std::exception &) {
    return fallback;
  }
}

inline long parseLongOr(const std::string &s, long fallback) {
  if (s.empty()) {
    return fallback;
  }
  try {
    return std::stol(s);
  }
  catch (const std::exception &) {
    return fallback;
  }
}

} // namespace mrs_uav_status::utils
