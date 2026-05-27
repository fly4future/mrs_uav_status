#pragma once

#include <exception>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/key_value.hpp>

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
