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
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic7_mocap_pose_latency.log";
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
    // Latency measurement: callback invocation time
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    // Parse frame_id
    std::string frame_id = pose->header.frame_id;
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

    Eigen::Quaterniond q_enu;

    tf2::fromMsg(pose->pose.orientation, q_enu);
    auto q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(q_enu));

    auto position = ftf::transform_frame_enu_ned(
      Eigen::Vector3d(
        pose->pose.position.x,
        pose->pose.position.y,
        pose->pose.position.z));

    mocap_pose_send(
      get_time_usec(pose->header.stamp),
      q,
      position);
    
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
