#pragma once

#include <string>
#include <vector>

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

} // namespace mrs_uav_status::utils
