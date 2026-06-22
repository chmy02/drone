/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ActuatorControl plugin
 * @file actuator_control.cpp
 * @author Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * @addtogroup plugin
 * @{
 */

#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/actuator_control.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief ActuatorControl plugin
 * @plugin actuator_control
 *
 * Sends actuator controls to FCU controller.
 */
class ActuatorControlPlugin : public plugin::Plugin
{
public:
  explicit ActuatorControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "actuator_control")
  {
    // Latency measurement log file (Topic 10)
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic10_actuator_control_latency.log";
    latency_log_file.open(ss.str(), std::ios::out | std::ios::app);
    if (latency_log_file.is_open()) {
      RCLCPP_INFO(get_logger(), "Latency log file opened: %s", ss.str().c_str());
    }

    auto sensor_qos = rclcpp::SensorDataQoS();

    target_actuator_control_pub = node->create_publisher<mavros_msgs::msg::ActuatorControl>(
      "target_actuator_control", sensor_qos);
    actuator_control_sub = node->create_subscription<mavros_msgs::msg::ActuatorControl>(
      "actuator_control", sensor_qos, std::bind(
        &ActuatorControlPlugin::actuator_control_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ActuatorControlPlugin::handle_actuator_control_target),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr target_actuator_control_pub;
  rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_control_sub;

  // Latency measurement (Topic 10)
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;

  /* -*- rx handlers -*- */

  void handle_actuator_control_target(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ACTUATOR_CONTROL_TARGET & act,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    auto ract = mavros_msgs::msg::ActuatorControl();
    ract.header.stamp = uas->synchronise_stamp(act.time_usec);
    ract.group_mix = act.group_mlx;
    ract.controls = act.controls;

    target_actuator_control_pub->publish(ract);
  }

  /* -*- callbacks -*- */

  void actuator_control_cb(const mavros_msgs::msg::ActuatorControl::SharedPtr req)
  {
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    std::string frame_id = req->header.frame_id;
    uint64_t publish_time_ns = 0;
    int node_id = 0;
    uint64_t msg_counter = 0;
    double cpu_total = 0.0, cpu_gz = 0.0, cpu_px4 = 0.0, cpu_mav = 0.0;
    
    size_t time_pos = frame_id.find("_time_");
    size_t cpu_pos = frame_id.find("_cpu_");
    if (time_pos != std::string::npos) {
      try {
        size_t node_pos = frame_id.find("node_");
        size_t msg_pos = frame_id.find("_msg_");
        if (node_pos != std::string::npos && msg_pos != std::string::npos) {
          node_id = std::stoi(frame_id.substr(node_pos + 5, msg_pos - node_pos - 5));
          msg_counter = std::stoull(frame_id.substr(msg_pos + 5, time_pos - msg_pos - 5));
        }
        size_t end_pos = (cpu_pos != std::string::npos) ? cpu_pos : frame_id.length();
        publish_time_ns = std::stoull(frame_id.substr(time_pos + 6, end_pos - time_pos - 6));
        if (cpu_pos != std::string::npos) {
          size_t gz_pos = frame_id.find("_gz_");
          size_t px4_pos = frame_id.find("_px4_");
          size_t mav_pos = frame_id.find("_mav_");
          if (gz_pos != std::string::npos) cpu_total = std::stod(frame_id.substr(cpu_pos + 5, gz_pos - cpu_pos - 5));
          if (px4_pos != std::string::npos) cpu_gz = std::stod(frame_id.substr(gz_pos + 4, px4_pos - gz_pos - 4));
          if (mav_pos != std::string::npos) cpu_px4 = std::stod(frame_id.substr(px4_pos + 5, mav_pos - px4_pos - 5));
          cpu_mav = std::stod(frame_id.substr(mav_pos + 5));
        }
      } catch (...) {}
    }

    mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET act{};
    act.time_usec = get_time_usec(req->header.stamp);
    act.group_mlx = req->group_mix;
    uas->msg_set_target(act);
    act.controls = req->controls;
    uas->send_message(act);

    auto send_complete_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    
    if (publish_time_ns > 0) {
      double t1_t3_us = (callback_start_ns - publish_time_ns) / 1000.0;
      double t3_t4_us = (send_complete_ns - callback_start_ns) / 1000.0;
      double t1_t4_us = (send_complete_ns - publish_time_ns) / 1000.0;
      
      std::lock_guard<std::mutex> lock(latency_log_mutex);
      if (latency_log_file.is_open()) {
        latency_log_file << node_id << "," << msg_counter << ","
                         << t1_t3_us << "," << t3_t4_us << "," << t1_t4_us << ","
                         << cpu_total << "," << cpu_gz << "," << cpu_px4 << "," << cpu_mav << "\n";
      }
    }
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ActuatorControlPlugin)
