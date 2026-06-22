/*
 * Copyright 2015 Matias Nitsche.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ManualControls plugin
 * @file manual_controls.cpp
 * @author Matias Nitsche <mnitsche@dc.uba.ar>
 *
 * @addtogroup plugin
 * @{
 */

#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>

#include <rcpputils/asserts.hpp>
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <mavros_msgs/msg/manual_control.hpp>

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Manual Control plugin
 * @plugin manual_control
 */
class ManualControlPlugin : public plugin::Plugin
{
public:
  explicit ManualControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "manual_control")
  {
    // Latency measurement log file (Topic 8)
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic8_manual_control_send_latency.log";
    latency_log_file.open(ss.str(), std::ios::out | std::ios::app);
    if (latency_log_file.is_open()) {
      RCLCPP_INFO(get_logger(), "Latency log file opened: %s", ss.str().c_str());
    }

    control_pub = node->create_publisher<mavros_msgs::msg::ManualControl>("~/control", 10);
    send_sub =
      node->create_subscription<mavros_msgs::msg::ManualControl>(
      "~/send", 10,
      std::bind(&ManualControlPlugin::send_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ManualControlPlugin::handle_manual_control),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr control_pub;
  rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr send_sub;

  // Latency measurement (Topic 8)
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;

  /* -*- rx handlers -*- */

  void handle_manual_control(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MANUAL_CONTROL & manual_control,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto manual_control_msg = mavros_msgs::msg::ManualControl();

    manual_control_msg.header.stamp = node->now();
    manual_control_msg.x = (manual_control.x / 1000.0);
    manual_control_msg.y = (manual_control.y / 1000.0);
    manual_control_msg.z = (manual_control.z / 1000.0);
    manual_control_msg.r = (manual_control.r / 1000.0);
    manual_control_msg.buttons = manual_control.buttons;

    manual_control_msg.buttons2 = manual_control.buttons2;
    manual_control_msg.enabled_extensions = manual_control.enabled_extensions;
    manual_control_msg.s = (manual_control.s / 1000.0);
    manual_control_msg.t = (manual_control.t / 1000.0);
    manual_control_msg.aux1 = (manual_control.aux1 / 1000.0);
    manual_control_msg.aux2 = (manual_control.aux2 / 1000.0);
    manual_control_msg.aux3 = (manual_control.aux3 / 1000.0);
    manual_control_msg.aux4 = (manual_control.aux4 / 1000.0);
    manual_control_msg.aux5 = (manual_control.aux5 / 1000.0);
    manual_control_msg.aux6 = (manual_control.aux6 / 1000.0);

    control_pub->publish(manual_control_msg);
  }

  /* -*- callbacks -*- */

  void send_cb(const mavros_msgs::msg::ManualControl::SharedPtr req)
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

    mavlink::common::msg::MANUAL_CONTROL msg = {};
    msg.target = uas->get_tgt_system();
    msg.x = req->x; msg.y = req->y; msg.z = req->z; msg.r = req->r;
    msg.buttons = req->buttons; msg.buttons2 = req->buttons2;
    msg.enabled_extensions = req->enabled_extensions;
    msg.s = req->s; msg.t = req->t;
    msg.aux1 = req->aux1; msg.aux2 = req->aux2; msg.aux3 = req->aux3;
    msg.aux4 = req->aux4; msg.aux5 = req->aux5; msg.aux6 = req->aux6;

    uas->send_message(msg);

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
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ManualControlPlugin)
