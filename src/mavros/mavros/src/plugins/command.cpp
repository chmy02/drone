/*
 * Copyright 2014,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Command plugin
 * @file command.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <future>
#include <chrono>
#include <condition_variable>
#include <list>
#include <memory>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/command_int.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_home.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_tol_local.hpp"
#include "mavros_msgs/srv/command_trigger_control.hpp"
#include "mavros_msgs/srv/command_trigger_interval.hpp"
#include "mavros_msgs/srv/command_vtol_transition.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

static constexpr std::chrono::nanoseconds ACK_TIMEOUT_DEFAULT = 5000ms;
using utils::enum_value;
using lock_guard = std::lock_guard<std::mutex>;
using unique_lock = std::unique_lock<std::mutex>;

class CommandTransaction
{
public:
  uint16_t expected_command;
  std::promise<uint8_t> promise;

  explicit CommandTransaction(uint16_t command)
  :    expected_command(command)
  {}
};

/**
 * @brief Command plugin.
 * @plugin command
 *
 * Send any command via COMMAND_LONG
 */
class CommandPlugin : public plugin::Plugin
{
public:
  explicit CommandPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "cmd"),
    use_comp_id_system_control(false),
    command_ack_timeout_dt(ACK_TIMEOUT_DEFAULT)
  {
    // Latency measurement log file (Topic 0 - Service)
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/rtcl-chmy/mavros_ws/src/mavros/chmy/logs/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_topic0_command_arm_latency.log";
    latency_log_file.open(ss.str(), std::ios::out | std::ios::app);
    if (latency_log_file.is_open()) {
      RCLCPP_INFO(get_logger(), "Latency log file opened: %s", ss.str().c_str());
    }

    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "command_ack_timeout", command_ack_timeout_dt.seconds(), [&](const rclcpp::Parameter & p) {
        command_ack_timeout_dt = rclcpp::Duration::from_seconds(p.as_double());
      });

    node_declare_and_watch_parameter(
      "use_comp_id_system_control", false, [&](const rclcpp::Parameter & p) {
        use_comp_id_system_control = p.as_bool();
      });

    srv_cg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

#ifdef USE_OLD_RMW_QOS
    auto services_qos = rmw_qos_profile_services_default;
#else
    auto services_qos = rclcpp::ServicesQoS();
#endif

    command_long_srv =
      node->create_service<mavros_msgs::srv::CommandLong>(
      "~/command",
      std::bind(
        &CommandPlugin::command_long_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    command_int_srv =
      node->create_service<mavros_msgs::srv::CommandInt>(
      "~/command_int",
      std::bind(
        &CommandPlugin::command_int_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    arming_srv =
      node->create_service<mavros_msgs::srv::CommandBool>(
      "~/arming",
      std::bind(
        &CommandPlugin::arming_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    set_home_srv =
      node->create_service<mavros_msgs::srv::CommandHome>(
      "~/set_home",
      std::bind(
        &CommandPlugin::set_home_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    takeoff_srv =
      node->create_service<mavros_msgs::srv::CommandTOL>(
      "~/takeoff",
      std::bind(
        &CommandPlugin::takeoff_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    takeoff_local_srv =
      node->create_service<mavros_msgs::srv::CommandTOLLocal>(
      "~/takeoff_local",
      std::bind(
        &CommandPlugin::takeoff_local_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    land_srv =
      node->create_service<mavros_msgs::srv::CommandTOL>(
      "~/land",
      std::bind(&CommandPlugin::land_cb, this, _1, _2, _3), services_qos,
      srv_cg);
    land_local_srv =
      node->create_service<mavros_msgs::srv::CommandTOLLocal>(
      "~/land_local",
      std::bind(&CommandPlugin::land_local_cb, this, _1, _2, _3), services_qos,
      srv_cg);
    trigger_control_srv = node->create_service<mavros_msgs::srv::CommandTriggerControl>(
      "~/trigger_control", std::bind(
        &CommandPlugin::trigger_control_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    trigger_interval_srv = node->create_service<mavros_msgs::srv::CommandTriggerInterval>(
      "~/trigger_interval", std::bind(
        &CommandPlugin::trigger_interval_cb, this, _1, _2,
        _3), services_qos, srv_cg);
    vtol_transition_srv = node->create_service<mavros_msgs::srv::CommandVtolTransition>(
      "~/vtol_transition", std::bind(
        &CommandPlugin::vtol_transition_cb, this, _1, _2,
        _3), services_qos, srv_cg);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&CommandPlugin::handle_command_ack)
    };
  }

private:
  using L_CommandTransaction = std::list<CommandTransaction>;

  std::mutex mutex;

  rclcpp::CallbackGroup::SharedPtr srv_cg;
  rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr command_long_srv;
  rclcpp::Service<mavros_msgs::srv::CommandInt>::SharedPtr command_int_srv;
  rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_srv;
  rclcpp::Service<mavros_msgs::srv::CommandHome>::SharedPtr set_home_srv;
  rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_srv;
  rclcpp::Service<mavros_msgs::srv::CommandTOLLocal>::SharedPtr takeoff_local_srv;
  rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr land_srv;
  rclcpp::Service<mavros_msgs::srv::CommandTOLLocal>::SharedPtr land_local_srv;
  rclcpp::Service<mavros_msgs::srv::CommandTriggerControl>::SharedPtr trigger_control_srv;
  rclcpp::Service<mavros_msgs::srv::CommandTriggerInterval>::SharedPtr trigger_interval_srv;
  rclcpp::Service<mavros_msgs::srv::CommandVtolTransition>::SharedPtr vtol_transition_srv;

  bool use_comp_id_system_control;

  L_CommandTransaction ack_waiting_list;
  rclcpp::Duration command_ack_timeout_dt;

  // Latency measurement (Topic 0 - Service)
  std::ofstream latency_log_file;
  std::mutex latency_log_mutex;
  uint64_t service_counter = 0;
  uint64_t arming_cb_invocation_count = 0;  // 콜백 호출 횟수 (throw 시에도 로그용)

  /* -*- message handlers -*- */

  void handle_command_ack(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::COMMAND_ACK & ack,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    lock_guard lock(mutex);

    for (auto & tr : ack_waiting_list) {
      if (tr.expected_command == ack.command) {
        tr.promise.set_value(ack.result);
        return;
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      10000, "CMD: Unexpected command %u, result %u",
      ack.command, ack.result);
  }

  /* -*- mid-level functions -*- */

  bool wait_ack_for(CommandTransaction & tr, uint8_t & result)
  {
    auto future = tr.promise.get_future();

    auto wres = future.wait_for(command_ack_timeout_dt.to_chrono<std::chrono::nanoseconds>());
    if (wres != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "CMD: Command %u -- ack timeout", tr.expected_command);
      return false;
    } else {
      result = future.get();
      return true;
    }
  }

  /**
   * Common function for command service callbacks.
   *
   * NOTE: success is bool in messages, but has unsigned char type in C++
   */
  void send_command_long_and_wait(
    bool broadcast,
    uint16_t command, uint8_t confirmation,
    float param1, float param2,
    float param3, float param4,
    float param5, float param6,
    float param7,
    bool & success, uint8_t & result)
  {
    using mavlink::common::MAV_RESULT;

    unique_lock lock(mutex);

    L_CommandTransaction::iterator ack_it;

    /* check transactions */
    for (const auto & tr : ack_waiting_list) {
      if (tr.expected_command == command) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 10000, "CMD: Command %u already in progress", command);
        throw std::logic_error("operation in progress");
      }
    }

    /**
     * @note APM & PX4 master always send COMMAND_ACK. Old PX4 never.
     * Don't expect any ACK in broadcast mode.
     */
    bool is_ack_required = (confirmation != 0 || uas->is_ardupilotmega() || uas->is_px4()) &&
      !broadcast;
    if (is_ack_required) {
      ack_it = ack_waiting_list.emplace(ack_waiting_list.end(), command);
    }

    command_long(
      broadcast,
      command, confirmation,
      param1, param2,
      param3, param4,
      param5, param6,
      param7);

    if (is_ack_required) {
      lock.unlock();
      bool is_not_timeout = wait_ack_for(*ack_it, result);
      lock.lock();

      success = is_not_timeout && result == enum_value(MAV_RESULT::ACCEPTED);

      ack_waiting_list.erase(ack_it);
    } else {
      success = true;
      result = enum_value(MAV_RESULT::ACCEPTED);
    }
  }

  /**
   * Common function for COMMAND_INT service callbacks.
   */
  void send_command_int(
    bool broadcast,
    uint8_t frame, uint16_t command,
    uint8_t current, uint8_t autocontinue,
    float param1, float param2,
    float param3, float param4,
    int32_t x, int32_t y,
    float z,
    bool & success)
  {
    /* Note: seems that COMMAND_INT don't produce COMMAND_ACK
     * so wait is not needed.
     */
    command_int(
      broadcast,
      frame, command, current, autocontinue,
      param1, param2,
      param3, param4,
      x, y, z);

    success = true;
  }

  /* -*- low-level send -*- */

  template<typename MsgT>
  inline void set_target(MsgT & cmd, bool broadcast)
  {
    using mavlink::minimal::MAV_COMPONENT;

    const uint8_t tgt_sys_id = (broadcast) ? 0 : uas->get_tgt_system();
    const uint8_t tgt_comp_id = (broadcast) ? 0 :
      (use_comp_id_system_control) ?
      enum_value(MAV_COMPONENT::COMP_ID_SYSTEM_CONTROL) : uas->get_tgt_component();

    cmd.target_system = tgt_sys_id;
    cmd.target_component = tgt_comp_id;
  }

  void command_long(
    bool broadcast,
    uint16_t command, uint8_t confirmation,
    float param1, float param2,
    float param3, float param4,
    float param5, float param6,
    float param7)
  {
    const uint8_t confirmation_fixed = (broadcast) ? 0 : confirmation;

    mavlink::common::msg::COMMAND_LONG cmd {};
    set_target(cmd, broadcast);

    cmd.command = command;
    cmd.confirmation = confirmation_fixed;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;

    uas->send_message(cmd);
  }

  void command_int(
    bool broadcast,
    uint8_t frame, uint16_t command,
    uint8_t current, uint8_t autocontinue,
    float param1, float param2,
    float param3, float param4,
    int32_t x, int32_t y,
    float z)
  {
    mavlink::common::msg::COMMAND_INT cmd {};
    set_target(cmd, broadcast);

    cmd.frame = frame;
    cmd.command = command;
    cmd.current = current;
    cmd.autocontinue = autocontinue;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.x = x;
    cmd.y = y;
    cmd.z = z;

    uas->send_message(cmd);
  }

  /* -*- callbacks -*- */

  void command_long_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandLong::Request::SharedPtr req,
    mavros_msgs::srv::CommandLong::Response::SharedPtr res)
  {
    // TODO(vooon): rewrite to use async service server
    send_command_long_and_wait(
      req->broadcast,
      req->command, req->confirmation,
      req->param1, req->param2,
      req->param3, req->param4,
      req->param5, req->param6,
      req->param7,
      res->success, res->result);
  }

  void command_int_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandInt::Request::SharedPtr req,
    mavros_msgs::srv::CommandInt::Response::SharedPtr res)
  {
    send_command_int(
      req->broadcast,
      req->frame, req->command,
      req->current, req->autocontinue,
      req->param1, req->param2,
      req->param3, req->param4,
      req->x, req->y, req->z,
      res->success);
  }

  void arming_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandBool::Request::SharedPtr req,
    mavros_msgs::srv::CommandBool::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    // 콜백 호출마다 카운트 (throw 시에도 로그 기록용)
    arming_cb_invocation_count++;
    int node_id = 0;
    uint64_t msg_counter = arming_cb_invocation_count;

    // t3: Service callback 시작 시각
    auto callback_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    // 요청에서 t1(timestamp_ns) 및 CPU 메타 파싱
    uint64_t publish_time_ns = req->timestamp_ns;
    double cpu_total = 0.0, cpu_gz = 0.0, cpu_px4 = 0.0, cpu_mav = 0.0;
    if (!req->cpu_meta.empty()) {
      try {
        std::string s = req->cpu_meta;
        size_t a = 0, b = s.find('_');
        if (b != std::string::npos) { cpu_total = std::stod(s.substr(a, b - a)); a = b + 1; b = s.find('_', a); }
        if (b != std::string::npos) { cpu_gz = std::stod(s.substr(a, b - a)); a = b + 1; b = s.find('_', a); }
        if (b != std::string::npos) { cpu_px4 = std::stod(s.substr(a, b - a)); a = b + 1; }
        cpu_mav = std::stod(s.substr(a));
      } catch (...) {}
    }

    using mavlink::common::MAV_RESULT;
    uint16_t command = enum_value(MAV_CMD::COMPONENT_ARM_DISARM);
    float param1 = (req->value) ? 1.0 : 0.0;

    unique_lock ack_lock(mutex);
    L_CommandTransaction::iterator ack_it;

    /* check transactions */
    for (const auto & tr : ack_waiting_list) {
      if (tr.expected_command == command) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 10000, "CMD: Command %u already in progress", command);
        // "operation in progress" 시에도 로그 한 줄 기록 (t3_t4=0, t1_t4=0), t1_t3·CPU는 요청 값 사용
        double t1_t3_reject = 0.0;
        if (publish_time_ns > 0) {
          t1_t3_reject = (static_cast<int64_t>(callback_start_ns) - static_cast<int64_t>(publish_time_ns)) / 1000.0;
        }
        {
          std::lock_guard<std::mutex> log_lock(latency_log_mutex);
          if (latency_log_file.is_open()) {
            latency_log_file << node_id << "," << msg_counter << ","
                             << t1_t3_reject << "," << 0.0 << "," << 0.0 << ","
                             << cpu_total << "," << cpu_gz << "," << cpu_px4 << "," << cpu_mav << "\n";
            latency_log_file.flush();
          }
        }
        throw std::logic_error("operation in progress");
      }
    }

    bool is_ack_required = uas->is_ardupilotmega() || uas->is_px4();
    if (is_ack_required) {
      ack_it = ack_waiting_list.emplace(ack_waiting_list.end(), command);
    }

    // command_long 호출 (send_message 완료까지)
    command_long(
      false,
      command, 1,
      param1, 0, 0, 0, 0, 0, 0);

    // t4: send_message() 완료 시각
    auto send_complete_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    // 레이턴시 계산 (마이크로초) - t1은 요청의 timestamp_ns
    double t1_t3_us = 0.0;
    double t3_t4_us = (send_complete_ns - callback_start_ns) / 1000.0;
    double t1_t4_us = t3_t4_us;
    if (publish_time_ns > 0) {
      t1_t3_us = (static_cast<int64_t>(callback_start_ns) - static_cast<int64_t>(publish_time_ns)) / 1000.0;
      t1_t4_us = (static_cast<int64_t>(send_complete_ns) - static_cast<int64_t>(publish_time_ns)) / 1000.0;
    }

    std::lock_guard<std::mutex> log_lock(latency_log_mutex);
    if (latency_log_file.is_open()) {
      latency_log_file << node_id << "," << msg_counter << ","
                       << t1_t3_us << "," << t3_t4_us << "," << t1_t4_us << ","
                       << cpu_total << "," << cpu_gz << "," << cpu_px4 << "," << cpu_mav << "\n";
      latency_log_file.flush();
    }

    // ACK 대기 및 응답 설정
    if (is_ack_required) {
      ack_lock.unlock();
      bool is_not_timeout = wait_ack_for(*ack_it, res->result);
      ack_lock.lock();
      res->success = is_not_timeout && res->result == enum_value(MAV_RESULT::ACCEPTED);
      ack_waiting_list.erase(ack_it);
    } else {
      res->success = true;
      res->result = enum_value(MAV_RESULT::ACCEPTED);
    }
  }

  void set_home_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandHome::Request::SharedPtr req,
    mavros_msgs::srv::CommandHome::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::DO_SET_HOME), 1,
      (req->current_gps) ? 1.0 : 0.0,
      0, 0, req->yaw, req->latitude, req->longitude, req->altitude,
      res->success, res->result);
  }

  void takeoff_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
    mavros_msgs::srv::CommandTOL::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::NAV_TAKEOFF), 1,
      req->min_pitch,
      0, 0,
      req->yaw,
      req->latitude, req->longitude, req->altitude,
      res->success, res->result);
  }

  void takeoff_local_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandTOLLocal::Request::SharedPtr req,
    mavros_msgs::srv::CommandTOLLocal::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::NAV_TAKEOFF_LOCAL), 1,
      req->min_pitch,
      0,
      req->rate,
      req->yaw,
      req->position.y, req->position.x, req->position.z,
      res->success, res->result);
  }

  void land_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandTOL::Request::SharedPtr req,
    mavros_msgs::srv::CommandTOL::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::NAV_LAND), 1,
      0, 0, 0,
      req->yaw,
      req->latitude, req->longitude, req->altitude,
      res->success, res->result);
  }

  void land_local_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandTOLLocal::Request::SharedPtr req,
    mavros_msgs::srv::CommandTOLLocal::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::NAV_LAND_LOCAL), 1,
      0,  // landing target number (what does it means?)
      req->offset,
      req->rate,
      req->yaw,
      req->position.y, req->position.x, req->position.z,
      res->success, res->result);
  }

  void trigger_control_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandTriggerControl::Request::SharedPtr req,
    mavros_msgs::srv::CommandTriggerControl::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::DO_TRIGGER_CONTROL), 1,
      (req->trigger_enable) ? 1.0 : 0.0,
      (req->sequence_reset) ? 1.0 : 0.0,
      (req->trigger_pause) ? 1.0 : 0.0,
      0, 0, 0, 0,
      res->success, res->result);
  }

  void trigger_interval_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandTriggerInterval::Request::SharedPtr req,
    mavros_msgs::srv::CommandTriggerInterval::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    // trigger interval can only be set when triggering is disabled
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::DO_SET_CAM_TRIGG_INTERVAL), 1,
      req->cycle_time,
      req->integration_time,
      0, 0, 0, 0, 0,
      res->success, res->result);
  }

  void vtol_transition_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const mavros_msgs::srv::CommandVtolTransition::Request::SharedPtr req,
    mavros_msgs::srv::CommandVtolTransition::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    send_command_long_and_wait(
      false,
      enum_value(MAV_CMD::DO_VTOL_TRANSITION), false,
      req->state,
      0, 0, 0, 0, 0, 0,
      res->success, res->result);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::CommandPlugin)
