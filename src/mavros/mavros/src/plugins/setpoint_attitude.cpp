/*
 * Copyright 2014 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointAttitude plugin
 * @file setpoint_attitude.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#if __has_include(<message_filters/subscriber.hpp>)
  #include <message_filters/subscriber.hpp>
  #include <message_filters/synchronizer.hpp>
  #include <message_filters/sync_policies/approximate_time.hpp>
#else
  #include <message_filters/subscriber.h>
  #include <message_filters/synchronizer.h>
  #include <message_filters/sync_policies/approximate_time.h>
#endif
#include <memory>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/thrust.hpp"

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

using SyncPoseThrustPolicy = message_filters::sync_policies::ApproximateTime<
  geometry_msgs::msg::PoseStamped, mavros_msgs::msg::Thrust>;
using SyncTwistThrustPolicy = message_filters::sync_policies::ApproximateTime<
  geometry_msgs::msg::TwistStamped, mavros_msgs::msg::Thrust>;
using SyncPoseThrust = message_filters::Synchronizer<SyncPoseThrustPolicy>;
using SyncTwistThrust = message_filters::Synchronizer<SyncTwistThrustPolicy>;

/**
 * @brief Setpoint attitude plugin
 * @plugin setpoint_attitude
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointAttitudePlugin : public plugin::Plugin,
  private plugin::SetAttitudeTargetMixin<SetpointAttitudePlugin>
{
public:
  explicit SetpointAttitudePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_attitude"),
    reverse_thrust(false)
  {
    enable_node_watch_parameters();

    auto qos = rclcpp::QoS(10);

    #ifdef USE_OLD_RMW_QOS_MESSAGE_FILTERS
    auto subscriber_qos = qos.get_rmw_qos_profile();
    #else
    auto subscriber_qos = qos;
    #endif

    node_declare_and_watch_parameter(
      "reverse_thrust", false, [&](const rclcpp::Parameter & p) {
        reverse_thrust = p.as_bool();
      });

    node_declare_and_watch_parameter(
      "use_quaternion", false, [&](const rclcpp::Parameter & p) {
        auto use_quaternion = p.as_bool();

        pose_sub.unsubscribe();
        twist_sub.unsubscribe();
        sync_pose.reset();
        sync_twist.reset();

        if (use_quaternion) {
          /**
           * @brief Use message_filters to sync attitude and thrust msg coming from different topics
           */
          pose_sub.subscribe(node, "~/attitude", subscriber_qos);

          sync_pose = std::make_unique<SyncPoseThrust>(SyncPoseThrustPolicy(10), pose_sub, th_sub);
          sync_pose->registerCallback(&SetpointAttitudePlugin::attitude_pose_cb, this);

        } else {
          twist_sub.subscribe(node, "~/cmd_vel", subscriber_qos);

          sync_twist =
          std::make_unique<SyncTwistThrust>(SyncTwistThrustPolicy(10), twist_sub, th_sub);
          sync_twist->registerCallback(&SetpointAttitudePlugin::attitude_twist_cb, this);
        }
      });

    // thrust msg subscriber to sync
    th_sub.subscribe(node, "~/thrust", subscriber_qos);

    // Latency measurement disabled for old Topic 5 (setpoint_attitude - not used)
    // 로그 파일 생성하지 않음
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetAttitudeTargetMixin<SetpointAttitudePlugin>;

  message_filters::Subscriber<mavros_msgs::msg::Thrust> th_sub;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub;
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> twist_sub;

  std::unique_ptr<SyncPoseThrust> sync_pose;
  std::unique_ptr<SyncTwistThrust> sync_twist;

  bool reverse_thrust;
  float normalized_thrust;

  // Latency measurement
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;

  /**
   * @brief Function to verify if the thrust values are normalized;
   * considers also the reversed trust values
   */
  inline bool is_normalized(float thrust)
  {
    auto lg = get_logger();

    if (reverse_thrust) {
      if (thrust < -1.0) {
        RCLCPP_WARN(lg, "Not normalized reversed thrust! Thd(%f) < Min(%f)", thrust, -1.0);
        return false;
      }
    } else {
      if (thrust < 0.0) {
        RCLCPP_WARN(lg, "Not normalized thrust! Thd(%f) < Min(%f)", thrust, 0.0);
        return false;
      }
    }

    if (thrust > 1.0) {
      RCLCPP_WARN(lg, "Not normalized thrust! Thd(%f) > Max(%f)", thrust, 1.0);
      return false;
    }
    return true;
  }

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send attitude setpoint and thrust to FCU attitude controller
   */
  void send_attitude_quaternion(
    const rclcpp::Time & stamp, const Eigen::Affine3d & tr,
    const float thrust)
  {
    /**
     * @note RPY, also bits numbering started from 1 in docs
     */
    const uint8_t ignore_all_except_q_and_thrust = (7 << 0);

    auto q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))
    );

    set_attitude_target(
      get_time_boot_ms(stamp),
      ignore_all_except_q_and_thrust,
      q,
      Eigen::Vector3d::Zero(),
      thrust);
  }

  /**
   * @brief Send angular velocity setpoint and thrust to FCU attitude controller
   */
  void send_attitude_ang_velocity(
    const rclcpp::Time & stamp, const Eigen::Vector3d & ang_vel,
    const float thrust)
  {
    /**
     * @note Q, also bits noumbering started from 1 in docs
     */
    const uint8_t ignore_all_except_rpy = (1 << 7);

    auto av = ftf::transform_frame_ned_enu(ang_vel);

    set_attitude_target(
      get_time_boot_ms(stamp),
      ignore_all_except_rpy,
      Eigen::Quaterniond::Identity(),
      av,
      thrust);
  }

  /* -*- callbacks -*- */

  void attitude_pose_cb(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg,
    const mavros_msgs::msg::Thrust::SharedPtr thrust_msg)
  {
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    std::string frame_id = pose_msg->header.frame_id;
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

    Eigen::Affine3d tr;
    tf2::fromMsg(pose_msg->pose, tr);

    if (is_normalized(thrust_msg->thrust)) {
      send_attitude_quaternion(pose_msg->header.stamp, tr, thrust_msg->thrust);
    }

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
        latency_log_file.flush();
      }
    }
  }

  void attitude_twist_cb(
    const geometry_msgs::msg::TwistStamped::SharedPtr req,
    const mavros_msgs::msg::Thrust::SharedPtr thrust_msg)
  {
    Eigen::Vector3d ang_vel;
    tf2::fromMsg(req->twist.angular, ang_vel);

    if (is_normalized(thrust_msg->thrust)) {
      send_attitude_ang_velocity(req->header.stamp, ang_vel, thrust_msg->thrust);
    }
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointAttitudePlugin)
