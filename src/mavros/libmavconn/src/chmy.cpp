//
// libmavconn
// Copyright 2024 chmy, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief MAVConn logging utilities for chmy
 * @file chmy.cpp
 * @author chmy
 *
 * @addtogroup mavconn
 * @{
 */

#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <mutex>

#include "mavconn/chmy.hpp"

namespace mavconn
{

static std::mutex log_mutex;
static const std::string LOG_FILE_PATH = "/home/rtcl-chmy/mavros_ws/src/mavros/libmavconn/log/emplace_pop_log.txt";

static std::string get_current_time()
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  
  std::ostringstream oss;
  oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
  oss << "." << std::setfill('0') << std::setw(3) << ms.count();
  
  return oss.str();
}

static void ensure_log_directory()
{
  std::filesystem::path log_path(LOG_FILE_PATH);
  std::filesystem::path log_dir = log_path.parent_path();
  
  if (!std::filesystem::exists(log_dir)) {
    std::filesystem::create_directories(log_dir);
  }
}

void chmy_emplace_log(uint32_t msgid)
{
  // Logging disabled
  return;
  
  // std::lock_guard<std::mutex> lock(log_mutex);
  // 
  // ensure_log_directory();
  // 
  // std::ofstream log_file(LOG_FILE_PATH, std::ios::app);
  // if (log_file.is_open()) {
  //   log_file << get_current_time() << " [emplace] " << msgid << std::endl;
  //   log_file.close();
  // }
}

void chmy_pop_log()
{
  // Logging disabled
  return;
  
  // std::lock_guard<std::mutex> lock(log_mutex);
  // 
  // ensure_log_directory();
  // 
  // std::ofstream log_file(LOG_FILE_PATH, std::ios::app);
  // if (log_file.is_open()) {
  //   log_file << get_current_time() << " [pop]" << std::endl;
  //   log_file.close();
  // }
}

}  // namespace mavconn
