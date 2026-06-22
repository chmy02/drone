/*
 * Copyright 2014 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointAcceleration plugin
 * @file setpoint_accel.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"

// Latency measurement
#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
#include <iomanip>

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Setpoint acceleration/force plugin
 * @plugin setpoint_accel
 *
 * Send setpoint accelerations/forces to FCU controller.
 */
class SetpointAccelerationPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointAccelerationPlugin>
{
public:
  explicit SetpointAccelerationPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_accel")
  {
    node->declare_parameter("send_force", false);

    auto sensor_qos = rclcpp::SensorDataQoS();

    accel_sub = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "~/accel", sensor_qos, std::bind(
        &SetpointAccelerationPlugin::accel_cb, this,
        _1));

    // Latency measurement log file initialization
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic5_setpoint_accel_accel_latency.log";
    latency_log_file.open(ss.str(), std::ios::out | std::ios::app);
    if (latency_log_file.is_open()) {
      RCLCPP_INFO(get_logger(), "Latency log: %s", ss.str().c_str());
    }
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointAccelerationPlugin>;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_sub;

  // Latency measurement
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send acceleration/force to FCU acceleration controller.
   *
   * @warning Send only AFX AFY AFZ. ENU frame.
   */
  void send_setpoint_acceleration(const rclcpp::Time & stamp, const Eigen::Vector3d & accel_enu)
  {
    using mavlink::common::MAV_FRAME;

    bool send_force;
    node->get_parameter("send_force", send_force);

    /* Documentation start from bit 1 instead 0.
     * Ignore position and velocity vectors, yaw and yaw rate
     */
    uint16_t ignore_all_except_a_xyz = (3 << 10) | (7 << 3) | (7 << 0);

    if (send_force) {
      ignore_all_except_a_xyz |= (1 << 9);
    }

    auto accel = ftf::transform_frame_enu_ned(accel_enu);

    set_position_target_local_ned(
      get_time_boot_ms(stamp),
      utils::enum_value(MAV_FRAME::LOCAL_NED),
      ignore_all_except_a_xyz,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      accel,
      0.0, 0.0);
  }

  /* -*- callbacks -*- */

  void accel_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr req)
  {
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    std::string frame_id = req->header.frame_id;
    int64_t publish_time_ns = 0;
    int node_id = 0;
    int msg_counter = 0;
    double cpu_total = 0.0, cpu_gz = 0.0, cpu_px4 = 0.0, cpu_mav = 0.0;

    size_t time_pos = frame_id.find("_time_");
    size_t cpu_pos = frame_id.find("_cpu_");
    if (time_pos != std::string::npos) {
      try {
        size_t node_pos = frame_id.find("node_");
        size_t msg_pos = frame_id.find("_msg_");
        if (node_pos != std::string::npos && msg_pos != std::string::npos) {
          node_id = std::stoi(frame_id.substr(node_pos + 5, msg_pos - node_pos - 5));
          msg_counter = std::stoi(frame_id.substr(msg_pos + 5, time_pos - msg_pos - 5));
        }
        size_t end_pos = (cpu_pos != std::string::npos) ? cpu_pos : frame_id.length();
        publish_time_ns = std::stoll(frame_id.substr(time_pos + 6, end_pos - time_pos - 6));
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

    Eigen::Vector3d accel_enu;
    tf2::fromMsg(req->vector, accel_enu);
    send_setpoint_acceleration(req->header.stamp, accel_enu);
    
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
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointAccelerationPlugin)
