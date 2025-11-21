/*
 * Copyright 2014 M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief VisionPoseEstimate plugin
 * @file vision_pose_estimate.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/utils.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/landing_target.hpp"

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
using namespace std::placeholders;

/**
 * @brief Vision pose estimate plugin
 * @plugin vision_pose
 *
 * Send pose estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisionPoseEstimatePlugin : public plugin::Plugin,
  private plugin::TF2ListenerMixin<VisionPoseEstimatePlugin>
{
public:
  explicit VisionPoseEstimatePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vision_pose"),
    tf_rate(10.0)
  {
    enable_node_watch_parameters();

    // tf params
    node_declare_and_watch_parameter(
      "tf/listen", false, [&](const rclcpp::Parameter & p) {
        auto tf_listen = p.as_bool();
        if (tf_listen) {
          RCLCPP_INFO_STREAM(
            get_logger(),
            "Listen to vision transform" << tf_frame_id <<
              " -> " << tf_child_frame_id);
          tf2_start("VisionPoseTF", &VisionPoseEstimatePlugin::transform_cb);
        } else {
          vision_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/pose", 10, std::bind(
              &VisionPoseEstimatePlugin::vision_cb, this, _1));
          vision_cov_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "~/pose_cov", 10, std::bind(
              &VisionPoseEstimatePlugin::vision_cov_cb, this, _1));
        }
      });

    node_declare_and_watch_parameter(
      "tf/frame_id", "map", [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });

    node_declare_and_watch_parameter(
      "tf/child_frame_id", "vision_estimate", [&](const rclcpp::Parameter & p) {
        tf_child_frame_id = p.as_string();
      });

    node_declare_and_watch_parameter(
      "tf/rate_limit", 10.0, [&](const rclcpp::Parameter & p) {
        tf_rate = p.as_double();
      });

    // Latency measurement log file initialization
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic6_vision_pose_pose_latency.log";
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
  friend class TF2ListenerMixin;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_cov_sub;

  std::string tf_frame_id;
  std::string tf_child_frame_id;

  double tf_rate;

  rclcpp::Time last_transform_stamp{0, 0, RCL_ROS_TIME};

  // Latency measurement
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;

  /* -*- low-level send -*- */
  /**
   * @brief Send vision estimate transform to FCU position controller
   */
  void send_vision_estimate(
    const rclcpp::Time & stamp, const Eigen::Affine3d & tr,
    const geometry_msgs::msg::PoseWithCovariance::_covariance_type & cov)
  {
    if (last_transform_stamp == stamp) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(), 10, "Vision: Same transform as last one, dropped.");
      return;
    }
    last_transform_stamp = stamp;

    auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
    auto rpy = ftf::quaternion_to_rpy(
      ftf::transform_orientation_enu_ned(
        ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))));

    auto cov_ned = ftf::transform_frame_enu_ned(cov);
    ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());

    mavlink::common::msg::VISION_POSITION_ESTIMATE vp{};

    vp.usec = stamp.nanoseconds() / 1000;
    // [[[cog:
    // for f in "xyz":
    //     cog.outl(f"vp.{f} = position.{f}();")
    // for a, b in zip("xyz", ('roll', 'pitch', 'yaw')):
    //     cog.outl(f"vp.{b} = rpy.{a}();")
    // ]]]
    vp.x = position.x();
    vp.y = position.y();
    vp.z = position.z();
    vp.roll = rpy.x();
    vp.pitch = rpy.y();
    vp.yaw = rpy.z();
    // [[[end]]] (checksum: 0aed118405958e3f35e8e7c9386e812f)

    // just the URT of the 6x6 Pose Covariance Matrix, given
    // that the matrix is symmetric
    ftf::covariance_urt_to_mavlink(cov_map, vp.covariance);

    uas->send_message(vp);
  }

  /* -*- callbacks -*- */

  /* common TF listener moved to mixin */

  void transform_cb(const geometry_msgs::msg::TransformStamped & transform)
  {
    Eigen::Affine3d tr = tf2::transformToEigen(transform.transform);
    ftf::Covariance6d cov {};                   // zero initialized

    send_vision_estimate(transform.header.stamp, tr, cov);
  }

  void vision_cb(const geometry_msgs::msg::PoseStamped::SharedPtr req)
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

    Eigen::Affine3d tr;
    tf2::fromMsg(req->pose, tr);
    ftf::Covariance6d cov {};                   // zero initialized

    send_vision_estimate(req->header.stamp, tr, cov);
    
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

  void vision_cov_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr req)
  {
    Eigen::Affine3d tr;
    tf2::fromMsg(req->pose.pose, tr);
    send_vision_estimate(req->header.stamp, tr, req->pose.covariance);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VisionPoseEstimatePlugin)
