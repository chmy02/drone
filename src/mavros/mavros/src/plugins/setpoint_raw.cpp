/**
 * @brief SetpointRAW plugin
 * @file setpoint_raw.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <algorithm>
#include <chrono>
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

#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/global_position_target.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Setpoint RAW plugin
 * @plugin setpoint_raw
 *
 * Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 */
class SetpointRawPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointRawPlugin>,
  private plugin::SetPositionTargetGlobalIntMixin<SetpointRawPlugin>,
  private plugin::SetAttitudeTargetMixin<SetpointRawPlugin>
{
public:
  explicit SetpointRawPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_raw"),
    thrust_scaling(1.0)
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "thrust_scaling", NAN, [&](const rclcpp::Parameter & p) {
        thrust_scaling = p.as_double();
      });

    // Latency measurement log file
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic1_setpoint_raw_local_latency.log";
    latency_log_file.open(ss.str(), std::ios::out | std::ios::app);
    if (latency_log_file.is_open()) {
      RCLCPP_INFO(get_logger(), "Latency log file opened: %s", ss.str().c_str());
    }

    // Attitude latency measurement log file
    std::stringstream ss_att;
    ss_att << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic4_setpoint_raw_attitude_latency.log";
    attitude_latency_log_file.open(ss_att.str(), std::ios::out | std::ios::app);
    if (attitude_latency_log_file.is_open()) {
      RCLCPP_INFO(get_logger(), "Attitude latency log file opened: %s", ss_att.str().c_str());
    }

    auto sensor_qos = rclcpp::SensorDataQoS();

    local_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
      "~/local", sensor_qos, std::bind(
        &SetpointRawPlugin::local_cb, this,
        _1));
    global_sub = node->create_subscription<mavros_msgs::msg::GlobalPositionTarget>(
      "~/global",
      sensor_qos, std::bind(
        &SetpointRawPlugin::global_cb, this,
        _1));
    attitude_sub = node->create_subscription<mavros_msgs::msg::AttitudeTarget>(
      "~/attitude",
      sensor_qos, std::bind(
        &SetpointRawPlugin::attitude_cb, this, _1));

    target_local_pub = node->create_publisher<mavros_msgs::msg::PositionTarget>(
      "~/target_local",
      sensor_qos);
    target_global_pub = node->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(
      "~/target_global", sensor_qos);
    target_attitude_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
      "~/target_attitude", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&SetpointRawPlugin::handle_position_target_local_ned),
      make_handler(&SetpointRawPlugin::handle_position_target_global_int),
      make_handler(&SetpointRawPlugin::handle_attitude_target),
    };
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointRawPlugin>;
  friend class plugin::SetPositionTargetGlobalIntMixin<SetpointRawPlugin>;
  friend class plugin::SetAttitudeTargetMixin<SetpointRawPlugin>;

  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr local_sub;
  rclcpp::Subscription<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_sub;
  rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_sub;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr target_local_pub;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr target_global_pub;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr target_attitude_pub;

  double thrust_scaling;

  // Latency measurement
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;
  std::atomic<uint64_t> msg_count{0};

  // Latency measurement for attitude
  std::ofstream attitude_latency_log_file;
  std::mutex attitude_latency_log_mutex;

  /* -*- message handlers -*- */
  void handle_position_target_local_ned(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::POSITION_TARGET_LOCAL_NED & tgt,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // Transform desired position,velocities,and accels from ENU to NED frame
    auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
    auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
    auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
    float yaw = ftf::quaternion_get_yaw(
      ftf::transform_orientation_aircraft_baselink(
        ftf::transform_orientation_ned_enu(
          ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
    Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
    auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
    float yaw_rate = ang_vel_enu.z();

    auto target = mavros_msgs::msg::PositionTarget();
    target.header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
    target.coordinate_frame = tgt.coordinate_frame;
    target.type_mask = tgt.type_mask;
    target.position = tf2::toMsg(position);
    tf2::toMsg(velocity, target.velocity);
    tf2::toMsg(af, target.acceleration_or_force);
    target.yaw = yaw;
    target.yaw_rate = yaw_rate;

    target_local_pub->publish(target);
  }

  void handle_position_target_global_int(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::POSITION_TARGET_GLOBAL_INT & tgt,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // Transform desired velocities from ENU to NED frame
    auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
    auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
    float yaw = ftf::quaternion_get_yaw(
      ftf::transform_orientation_aircraft_baselink(
        ftf::transform_orientation_ned_enu(
          ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
    Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
    auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
    float yaw_rate = ang_vel_enu.z();

    auto target = mavros_msgs::msg::GlobalPositionTarget();
    target.header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
    target.coordinate_frame = tgt.coordinate_frame;
    target.type_mask = tgt.type_mask;
    target.latitude = tgt.lat_int / 1e7;
    target.longitude = tgt.lon_int / 1e7;
    target.altitude = tgt.alt;
    tf2::toMsg(velocity, target.velocity);
    tf2::toMsg(af, target.acceleration_or_force);
    target.yaw = yaw;
    target.yaw_rate = yaw_rate;

    target_global_pub->publish(target);
  }

  void handle_attitude_target(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ATTITUDE_TARGET & tgt,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // Transform orientation from baselink -> ENU
    // to aircraft -> NED
    auto orientation = ftf::transform_orientation_ned_enu(
      ftf::transform_orientation_baselink_aircraft(
        ftf::mavlink_to_quaternion(tgt.q)));

    auto body_rate =
      ftf::transform_frame_baselink_aircraft(
      Eigen::Vector3d(
        tgt.body_roll_rate,
        tgt.body_pitch_rate, tgt.body_yaw_rate));

    auto target = mavros_msgs::msg::AttitudeTarget();
    target.header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
    target.type_mask = tgt.type_mask;
    target.orientation = tf2::toMsg(orientation);
    tf2::toMsg(body_rate, target.body_rate);
    target.thrust = tgt.thrust;

    target_attitude_pub->publish(target);
  }

  /* -*- callbacks -*- */

  void local_cb(const mavros_msgs::msg::PositionTarget::SharedPtr req)
  {
    // Latency measurement: callback 시작 시각 (나노초)
    // system_clock 사용 (Python의 time.time_ns()와 동일한 기준)
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    
    using mavros_msgs::msg::PositionTarget;

    Eigen::Vector3d position, velocity, af;
    float yaw, yaw_rate;

    // Parse timestamp from frame_id if available
    // Format: "node_{node_id}_msg_{counter}_time_{publish_time_ns}"
    uint64_t publish_time_ns = 0;
    int node_id = 0;
    uint64_t msg_counter = 0;
    
    if (req->header.frame_id.find("_time_") != std::string::npos) {
      // Extract publish time from frame_id
      try {
        size_t time_pos = req->header.frame_id.find("_time_");
        if (time_pos != std::string::npos) {
          std::string time_str = req->header.frame_id.substr(time_pos + 6);
          publish_time_ns = std::stoull(time_str);
          
          // Extract node_id and msg_counter
          size_t node_pos = req->header.frame_id.find("node_");
          size_t msg_pos = req->header.frame_id.find("_msg_");
          if (node_pos != std::string::npos && msg_pos != std::string::npos) {
            node_id = std::stoi(req->header.frame_id.substr(node_pos + 5, msg_pos - node_pos - 5));
            size_t msg_end = req->header.frame_id.find("_time_");
            if (msg_end != std::string::npos) {
              msg_counter = std::stoull(req->header.frame_id.substr(msg_pos + 5, msg_end - msg_pos - 5));
            }
          }
          
          // Latency will be logged after send_message() completes (at t4)
        }
      } catch (const std::exception& e) {
        // Ignore parsing errors
      }
    }

    tf2::fromMsg(req->position, position);
    tf2::fromMsg(req->velocity, velocity);
    tf2::fromMsg(req->acceleration_or_force, af);

    // Transform frame ENU->NED
    if (req->coordinate_frame == PositionTarget::FRAME_BODY_NED ||
      req->coordinate_frame == PositionTarget::FRAME_BODY_OFFSET_NED)
    {
      position = ftf::transform_frame_baselink_aircraft(position);
      velocity = ftf::transform_frame_baselink_aircraft(velocity);
      af = ftf::transform_frame_baselink_aircraft(af);
      yaw = ftf::quaternion_get_yaw(
        ftf::transform_orientation_absolute_frame_aircraft_baselink(
          ftf::quaternion_from_rpy(0.0, 0.0, req->yaw)));
    } else {
      position = ftf::transform_frame_enu_ned(position);
      velocity = ftf::transform_frame_enu_ned(velocity);
      af = ftf::transform_frame_enu_ned(af);
      yaw = ftf::quaternion_get_yaw(
        ftf::transform_orientation_aircraft_baselink(
          ftf::transform_orientation_ned_enu(
            ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
    }

    Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
    auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
    yaw_rate = ang_vel_ned.z();

    set_position_target_local_ned(
      get_time_boot_ms(
        req->header.stamp),
      req->coordinate_frame,
      req->type_mask,
      position,
      velocity,
      af,
      yaw, yaw_rate);
    
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

  void global_cb(const mavros_msgs::msg::GlobalPositionTarget::SharedPtr req)
  {
    Eigen::Vector3d velocity, af;
    float yaw, yaw_rate;

    tf2::fromMsg(req->velocity, velocity);
    tf2::fromMsg(req->acceleration_or_force, af);

    // Transform frame ENU->NED
    velocity = ftf::transform_frame_enu_ned(velocity);
    af = ftf::transform_frame_enu_ned(af);
    yaw = ftf::quaternion_get_yaw(
      ftf::transform_orientation_aircraft_baselink(
        ftf::transform_orientation_ned_enu(
          ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
    Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
    auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
    yaw_rate = ang_vel_ned.z();

    set_position_target_global_int(
      get_time_boot_ms(req->header.stamp),
      req->coordinate_frame,
      req->type_mask,
      req->latitude * 1e7,
      req->longitude * 1e7,
      req->altitude,
      velocity,
      af,
      yaw, yaw_rate);
  }

  void attitude_cb(const mavros_msgs::msg::AttitudeTarget::SharedPtr req)
  {
    // Latency measurement: callback invocation time
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    // Parse frame_id: "node_<id>_msg_<counter>_time_<publish_time_ns>"
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

    Eigen::Quaterniond desired_orientation;
    Eigen::Vector3d baselink_angular_rate;
    Eigen::Vector3d body_rate;
    double thrust;

    // Set Thrust scaling in px4_config.yaml, setpoint_raw block.
    // ignore thrust is false by default, unless no thrust scaling is set or thrust is zero
    auto ignore_thrust = req->thrust != 0.0 && std::isnan(thrust_scaling);

    if (ignore_thrust) {
      // I believe it's safer without sending zero thrust, but actually ignoring the actuation.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Recieved thrust, but ignore_thrust is true: "
        "the most likely cause of this is a failure to specify the thrust_scaling parameters "
        "on px4/apm_config.yaml. Actuation will be ignored.");
      return;
    } else {
      if (thrust_scaling == 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "thrust_scaling parameter is set to zero.");
      }
      if (std::isnan(thrust_scaling)) {
        thrust_scaling = 1.0;
      }
      thrust = std::min(1.0, std::max(0.0, req->thrust * thrust_scaling));

      // Take care of attitude setpoint
      desired_orientation = ftf::to_eigen(req->orientation);

      // Transform desired orientation to represent aircraft->NED,
      // MAVROS operates on orientation of base_link->ENU
      auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
        ftf::transform_orientation_baselink_aircraft(desired_orientation));

      body_rate = ftf::transform_frame_baselink_aircraft(
        ftf::to_eigen(req->body_rate));

      set_attitude_target(
        get_time_boot_ms(req->header.stamp),
        req->type_mask,
        ned_desired_orientation,
        body_rate,
        thrust);
      
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
        
        std::lock_guard<std::mutex> lock(attitude_latency_log_mutex);
        if (attitude_latency_log_file.is_open()) {
          attitude_latency_log_file << node_id << ","
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
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointRawPlugin)
