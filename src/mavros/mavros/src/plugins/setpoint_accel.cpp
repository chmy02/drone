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
    // Latency measurement: callback invocation time
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    // Parse frame_id
    std::string frame_id = req->header.frame_id;
    int64_t publish_time_ns = 0;
    int node_id = 0;
    int msg_counter = 0;

    size_t time_pos = frame_id.find("_time_");
    if (time_pos != std::string::npos) {
      publish_time_ns = std::stoll(frame_id.substr(time_pos + 6));
      
      size_t node_pos = frame_id.find("node_");
      size_t msg_pos = frame_id.find("_msg_");
      if (node_pos != std::string::npos && msg_pos != std::string::npos) {
        node_id = std::stoi(frame_id.substr(node_pos + 5, msg_pos - node_pos - 5));
        msg_counter = std::stoi(frame_id.substr(msg_pos + 5, time_pos - msg_pos - 5));
      }

      // Latency will be logged after send_message() completes (at t4)
    }

    Eigen::Vector3d accel_enu;

    tf2::fromMsg(req->vector, accel_enu);
    send_setpoint_acceleration(req->header.stamp, accel_enu);
    
    // t4: send_message() 완료 시각 측정
    auto send_complete_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    
    // t3 → t4: MAVROS 내부 처리 시간
    int64_t processing_latency_ns = send_complete_ns - callback_start_ns;
    double processing_latency_us = processing_latency_ns / 1e3;  // 마이크로초
    
    // 로그 업데이트 (t4 추가)
    if (publish_time_ns > 0) {
      int64_t total_latency_ns = callback_start_ns - publish_time_ns;
      double total_latency_us = total_latency_ns / 1e3;  // 마이크로초
      
      std::lock_guard<std::mutex> lock(latency_log_mutex);
      if (latency_log_file.is_open()) {
        latency_log_file << node_id << ","
                         << msg_counter << ","
                         << publish_time_ns << ","
                         << callback_start_ns << ","
                         << send_complete_ns << ","
                         << processing_latency_ns << ","
                         << processing_latency_us << ","
                         << total_latency_ns << ","
                         << total_latency_us << "\n";
      }
    }
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointAccelerationPlugin)
