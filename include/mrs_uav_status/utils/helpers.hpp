#pragma once

/* includes //{ */

#include <cstdint>
#include <exception>
#include <string>
#include <vector>

//}

namespace mrs_uav_status::utils
{

// Mirrors mrs_msgs::msg::GeneralRobotInfo::ROBOT_TYPE_* -- named locally so utils/ keeps no
// dependency on ROS message types.
inline constexpr uint8_t ROBOT_TYPE_DRONE        = 0;
inline constexpr uint8_t ROBOT_TYPE_BOAT         = 1;
inline constexpr uint8_t ROBOT_TYPE_GROUND_ROBOT = 2;

// Splits input on delimiter (returns {""} for an empty input).
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
  case ROBOT_TYPE_DRONE:
    return "DRONE";
  case ROBOT_TYPE_BOAT:
    return "BOAT";
  case ROBOT_TYPE_GROUND_ROBOT:
    return "UGV";
  default:
    return "UNKNOWN";
  }
}

// Find the first SensorStatus of a given type. Returns nullptr if not present.
// Templated so this works for status::SensorStatusData without a dependency on status/ or mrs_msgs.
template <typename SensorT>
inline const SensorT *findSensor(const std::vector<SensorT> &sensors, uint8_t type) {
  for (const auto &s : sensors) {
    if (s.type == type) {
      return &s;
    }
  }
  return nullptr;
}

// Look up a value in a KeyValue list. Returns fallback if the key isn't present.
// Templated for the same reason as findSensor() above -- works for both
// diagnostic_msgs::msg::KeyValue and status::KeyValueData.
template <typename KeyValueT>
inline std::string lookupDetail(const std::vector<KeyValueT> &details, const std::string &key, const std::string &fallback = "") {
  for (const auto &kv : details) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return fallback;
}

// Parses s as a double; returns fallback if empty or unparseable.
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

// Parses s as a long; returns fallback if empty or unparseable.
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
