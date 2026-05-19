#include <mrs_uav_status/ros/topic_info.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status
{

//}

/* TopicInfo() //{ */

TopicInfo::TopicInfo(rclcpp::Node::SharedPtr node, double window_rate, int buffer_length, double desired_rate)
    : TopicInfo(node, window_rate, buffer_length, desired_rate, "NOT_DEFINED", "NOT_DEFINED") {
}

//}

/* TopicInfo() //{ */

TopicInfo::TopicInfo(rclcpp::Node::SharedPtr node, double window_rate, int buffer_length, double desired_rate, const std::string &topic_name,
                     const std::string &topic_display_name)
    : node_(node), window_rate_(window_rate), desired_rate_(desired_rate), topic_name_(topic_name), topic_display_name_(topic_display_name),
      last_time_(0, 0, node->get_clock()->get_clock_type()), counter_(0), rates_(static_cast<size_t>(buffer_length * int(window_rate)), 0.0),
      rates_iterator_(0) {
}

//}

/* getHz //{ */

std::tuple<double, int16_t> TopicInfo::getHz() {

  rclcpp::Time time_now = node_->get_clock()->now();
  double       interval = (time_now - last_time_).seconds();

  if (interval == 0.0) {
    return std::make_tuple(0.0, static_cast<int16_t>(tui::ColorPair::Red));
  }

  last_time_ = time_now;

  double avg_rate = counter_ / interval;
  counter_        = 0;

  rates_[rates_iterator_] = avg_rate;
  rates_iterator_++;

  if (rates_iterator_ >= rates_.size()) {
    rates_iterator_ = 0;
  }

  avg_rate = 0.0;

  for (unsigned long i = 0; i < rates_.size(); i++) {
    avg_rate += rates_[i];
  }

  if (rates_.size() == 0) {
    avg_rate = 0.0;
  } else {
    avg_rate = avg_rate / double(rates_.size());
  }

  int16_t color = static_cast<int16_t>(tui::ColorPair::Red);

  if (avg_rate > 0.9 * desired_rate_) {
    color = static_cast<int16_t>(tui::ColorPair::Green);
  } else if (avg_rate > 0.5 * desired_rate_) {
    color = static_cast<int16_t>(tui::ColorPair::Yellow);
  }

  return std::make_tuple(avg_rate, color);
}

//}

/* getTopicName //{ */

std::string TopicInfo::getTopicName() const {
  return topic_name_;
}

//}

/* getTopicDisplayName //{ */

std::string TopicInfo::getTopicDisplayName() const {
  return topic_display_name_;
}

//}

/* count //{ */

void TopicInfo::count() {
  counter_++;
}

//}

} // namespace mrs_uav_status
