/*
 * Copyright 2014 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointVelocity plugin
 * @file setpoint_velocity.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <chrono>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <atomic>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::MAV_FRAME;

/**
 * @brief Setpoint velocity plugin
 * @plugin setpoint_velocity
 *
 * Send setpoint velocities to FCU controller.
 */
class SetpointVelocityPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin>
{
public:
  explicit SetpointVelocityPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_velocity")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "mav_frame", "LOCAL_NED", [&](const rclcpp::Parameter & p) {
        auto mav_frame_str = p.as_string();
        auto new_mav_frame = utils::mav_frame_from_str(mav_frame_str);

        if (new_mav_frame == MAV_FRAME::LOCAL_NED && mav_frame_str != "LOCAL_NED") {
          throw rclcpp::exceptions::InvalidParameterValueException(
            utils::format(
              "unknown MAV_FRAME: %s",
              mav_frame_str.c_str()));
        }
        mav_frame = new_mav_frame;
      });

    // Latency measurement log file
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic2_setpoint_velocity_cmd_vel_latency.log";
    latency_log_file.open(ss.str(), std::ios::out | std::ios::app);
    if (latency_log_file.is_open()) {
      RCLCPP_INFO(get_logger(), "Velocity latency log file opened: %s", ss.str().c_str());
    }

    std::stringstream ss_e2e;
    ss_e2e << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/" <<
      std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") <<
      "_obstacle_stop_mavros_t4.log";
    obstacle_e2e_t4_log.open(ss_e2e.str(), std::ios::out | std::ios::app);
    if (obstacle_e2e_t4_log.is_open()) {
      obstacle_e2e_t4_log <<
        "# obstacle_stop E2E: lidar_corr_stamp_ns t_mavros_cb_ros_ns "
        "t4_after mavlink_send_ros_ns (ROS clock ns; Python t1_ns/t5와 동일 축으로 use_sim_time 권장)\n";
      RCLCPP_INFO(
        get_logger(), "Obstacle E2E t4 log opened: %s",
        ss_e2e.str().c_str());
    }

    auto sensor_qos = rclcpp::SensorDataQoS();

    // cmd_vel usually is the topic used for velocity control in many controllers / planners
    vel_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel", sensor_qos, std::bind(
        &SetpointVelocityPlugin::vel_cb, this,
        _1));
    vel_unstamped_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel_unstamped",
      sensor_qos, std::bind(
        &SetpointVelocityPlugin::vel_unstamped_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin>;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_unstamped_sub;

  MAV_FRAME mav_frame;

  // Latency measurement
  std::ofstream latency_log_file;
  std::ofstream obstacle_e2e_t4_log;
  std::mutex latency_log_mutex;
  std::atomic<uint64_t> msg_count{0};

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send velocity to FCU velocity controller
   *
   * @warning Send only VX VY VZ. ENU frame.
   */
  void send_setpoint_velocity(
    const rclcpp::Time & stamp, const Eigen::Vector3d & vel_enu,
    const double yaw_rate)
  {
    /**
     * Documentation start from bit 1 instead 0;
     * Ignore position and accel vectors, yaw.
     */
    uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);
    auto vel = [&]() {
        if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
          return ftf::transform_frame_baselink_aircraft(vel_enu);
        } else {
          return ftf::transform_frame_enu_ned(vel_enu);
        }
      } ();

    auto yr = [&]() {
        if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
          return ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(0.0, 0.0, yaw_rate));
        } else {
          return ftf::transform_frame_ned_enu(Eigen::Vector3d(0.0, 0.0, yaw_rate));
        }
      } ();

    set_position_target_local_ned(
      get_time_boot_ms(stamp),
      utils::enum_value(mav_frame),
      ignore_all_except_v_xyz_yr,
      Eigen::Vector3d::Zero(),
      vel,
      Eigen::Vector3d::Zero(),
      0.0, yr.z());
  }

  /* -*- callbacks -*- */

  void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr req)
  {
    const int64_t t_mavros_cb_ros_ns = node->now().nanoseconds();

    auto t3_callback_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    
    uint64_t t1_publish_ns = static_cast<uint64_t>(req->header.stamp.sec) * 1000000000ULL 
                           + static_cast<uint64_t>(req->header.stamp.nanosec);

    int node_id = 2;
    uint64_t msg_counter = 0;
    double cpu_total = 0.0, cpu_gz = 0.0, cpu_px4 = 0.0, cpu_mav = 0.0;
    
    std::string frame_id = req->header.frame_id;
    try {
      size_t node_pos = frame_id.find("node_");
      size_t msg_pos = frame_id.find("_msg_");
      size_t time_pos = frame_id.find("_time_");
      size_t cpu_pos = frame_id.find("_cpu_");
      
      if (node_pos != std::string::npos && msg_pos != std::string::npos)
        node_id = std::stoi(frame_id.substr(node_pos + 5, msg_pos - node_pos - 5));
      if (msg_pos != std::string::npos && time_pos != std::string::npos)
        msg_counter = std::stoull(frame_id.substr(msg_pos + 5, time_pos - msg_pos - 5));
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

    Eigen::Vector3d vel_enu;
    tf2::fromMsg(req->twist.linear, vel_enu);
    send_setpoint_velocity(req->header.stamp, vel_enu, req->twist.angular.z);
    
    const int64_t t4_after_send_ros_ns = node->now().nanoseconds();

    // obstacle_e2e 또는 동일 접두사+패딩(cmd_vel 크기 확대 실험)
    static constexpr const char kObstacleE2e[] = "obstacle_e2e";
    const bool obstacle_e2e_tag =
      (frame_id.size() >= sizeof(kObstacleE2e) - 1) &&
      (frame_id.compare(0, sizeof(kObstacleE2e) - 1, kObstacleE2e) == 0);

    if (obstacle_e2e_tag) {
      std::lock_guard<std::mutex> lock(latency_log_mutex);
      if (obstacle_e2e_t4_log.is_open()) {
        obstacle_e2e_t4_log << t1_publish_ns << ',' << t_mavros_cb_ros_ns << ','
                            << t4_after_send_ros_ns << '\n';
        obstacle_e2e_t4_log.flush();
      }
    }

    auto t4_send_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    
    if (t1_publish_ns > 1000000000000000000ULL) {
      double t1_t3_us = (t3_callback_ns - t1_publish_ns) / 1000.0;
      double t3_t4_us = (t4_send_ns - t3_callback_ns) / 1000.0;
      double t1_t4_us = (t4_send_ns - t1_publish_ns) / 1000.0;
      
      std::lock_guard<std::mutex> lock(latency_log_mutex);
      if (latency_log_file.is_open()) {
        latency_log_file << node_id << "," << msg_counter << ","
                         << t1_t3_us << "," << t3_t4_us << "," << t1_t4_us << ","
                         << cpu_total << "," << cpu_gz << "," << cpu_px4 << "," << cpu_mav << "\n";
        latency_log_file.flush();
      }
    }
  }

  void vel_unstamped_cb(const geometry_msgs::msg::Twist::SharedPtr req)
  {
    Eigen::Vector3d vel_enu;

    tf2::fromMsg(req->linear, vel_enu);
    send_setpoint_velocity(
      node->now(), vel_enu,
      req->angular.z);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointVelocityPlugin)
