/*
 * Copyright 2014,2015,2016 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Latency measurement
#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
#include <iomanip>

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief MocapPoseEstimate plugin
 * @plugin mocap_pose_estimate
 *
 * Sends motion capture data to FCU.
 */
class MocapPoseEstimatePlugin : public plugin::Plugin
{
public:
  explicit MocapPoseEstimatePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "mocap")
  {
    /** @note For VICON ROS package, subscribe to TransformStamped topic */
    mocap_tf_sub = node->create_subscription<geometry_msgs::msg::TransformStamped>(
      "~/tf", 1, std::bind(
        &MocapPoseEstimatePlugin::mocap_tf_cb, this,
        _1));
    /** @note For Optitrack ROS package, subscribe to PoseStamped topic */
    mocap_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/pose", 1, std::bind(
        &MocapPoseEstimatePlugin::mocap_pose_cb, this,
        _1));

    // Latency measurement log file initialization
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic12_mocap_pose_latency.log";
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr mocap_tf_sub;

  // Latency measurement
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;

  /* -*- low-level send -*- */
  void mocap_pose_send(
    uint64_t usec,
    Eigen::Quaterniond & q,
    Eigen::Vector3d & v)
  {
    mavlink::common::msg::ATT_POS_MOCAP pos = {};

    pos.time_usec = usec;
    ftf::quaternion_to_mavlink(q, pos.q);
    pos.x = v.x();
    pos.y = v.y();
    pos.z = v.z();

    uas->send_message(pos);
  }

  /* -*- callbacks -*- */

  void mocap_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    std::string frame_id = pose->header.frame_id;
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

    Eigen::Quaterniond q_enu;
    tf2::fromMsg(pose->pose.orientation, q_enu);
    auto q = ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(q_enu));
    auto position = ftf::transform_frame_enu_ned(
      Eigen::Vector3d(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));

    mocap_pose_send(get_time_usec(pose->header.stamp), q, position);
    
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

  void mocap_tf_cb(const geometry_msgs::msg::TransformStamped::SharedPtr trans)
  {
    Eigen::Quaterniond q_enu;

    tf2::fromMsg(trans->transform.rotation, q_enu);
    auto q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(q_enu));

    auto position = ftf::transform_frame_enu_ned(
      Eigen::Vector3d(
        trans->transform.translation.x,
        trans->transform.translation.y,
        trans->transform.translation.z));

    mocap_pose_send(
      get_time_usec(trans->header.stamp),
      q,
      position);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::MocapPoseEstimatePlugin)
