//
// libmavconn
// Copyright 2013,2014,2015,2016,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief CHMY utility functions collection
 * @file chmy.cpp
 * @author CHMY
 *
 * @addtogroup mavconn
 * @{
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <unordered_map>
#include <mutex>
#include <thread>

namespace mavconn
{

// ============================================================================
// 여기에 앞으로 추가하고 싶은 함수들을 정리해두세요
// ============================================================================

/**
 * @brief 예시 함수 - 필요에 따라 수정하거나 삭제하세요
 */
void example_function() {
    std::cout << "CHMY utility function example" << std::endl;
}

/**
 * @brief MAVLink 메시지 ID를 문자열로 변환하는 함수
 * @param msg_id MAVLink 메시지 ID
 * @return 메시지 이름 문자열
 */
std::string get_mavlink_message_name(uint32_t msg_id) {
    // TODO: 구현 필요
    return "Unknown Message ID: " + std::to_string(msg_id);
}

/**
 * @brief 시스템 상태를 확인하는 함수
 * @return 시스템 상태 문자열
 */
std::string check_system_status() {
    // TODO: 구현 필요
    return "System Status: OK";
}

// ============================================================================
// 추가할 함수들을 여기에 계속 작성하세요
// ============================================================================

// 전역 변수로 명령별 시작 시간을 저장
static std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> command_start_times;
static std::mutex command_times_mutex;

// 스레드 추적을 위한 전역 변수들
static std::unordered_map<std::string, std::unordered_map<uint64_t, size_t>> mutex_lock_counts;
static std::unordered_map<std::string, std::unordered_map<uint64_t, std::chrono::steady_clock::time_point>> mutex_last_lock_times;
static std::mutex thread_stats_mutex;

/**
 * @brief 로그 파일 경로를 반환하는 헬퍼 함수
 * @return 로그 파일의 전체 경로
 */
std::string get_log_file_path() {
    return "/home/rtcl-chmy/mavros_ws/src/mavros/libmavconn/log/txq_timer_log.txt";
}

/**
 * @brief 로그 디렉토리를 생성하는 헬퍼 함수
 */
void ensure_log_directory() {
    std::string log_path = get_log_file_path();
    std::filesystem::path file_path(log_path);
    std::filesystem::path dir_path = file_path.parent_path();
    
    if (!std::filesystem::exists(dir_path)) {
        std::filesystem::create_directories(dir_path);
    }
}

/**
 * @brief 현재 시간을 포맷된 문자열로 반환하는 헬퍼 함수
 * @return "MM-DD HH:MM:SS" 형식의 시간 문자열
 */
std::string get_current_time_string() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()) % 1000000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%m-%d %H:%M:%S");
    ss << " " << std::setfill('0') << std::setw(6) << ms.count();
    
    return ss.str();
}

/**
 * @brief txq emplace 함수 실행 시점을 로깅하는 함수
 * @param command_id 명령 ID
 * @param queue_size 현재 큐 크기
 */
void chmy_emplace_log(uint32_t command_id, size_t queue_size) {
    std::lock_guard<std::mutex> lock(command_times_mutex);
    
    // 명령 시작 시간을 기록
    command_start_times[command_id] = std::chrono::steady_clock::now();
    
    // 로그 파일에 기록
    ensure_log_directory();
    std::ofstream log_file(get_log_file_path(), std::ios::app);
    
    if (log_file.is_open()) {
        log_file << get_current_time_string() << " [Emplace] " 
                 << command_id << " " << queue_size << std::endl;
        log_file.close();
    }
}

/**
 * @brief txq popfront 함수 실행 시점을 로깅하는 함수
 * @param command_id 명령 ID
 * @param queue_size 현재 큐 크기
 */
void chmy_popfront_log(uint32_t command_id, size_t queue_size) {
    std::lock_guard<std::mutex> lock(command_times_mutex);
    
    // 명령 시작 시간을 찾아서 경과 시간 계산
    auto it = command_start_times.find(command_id);
    if (it != command_start_times.end()) {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - it->second);
        
        // 로그 파일에 기록
        ensure_log_directory();
        std::ofstream log_file(get_log_file_path(), std::ios::app);
        
        if (log_file.is_open()) {
            log_file << get_current_time_string() << " [PopFront] " 
                     << command_id << " " << queue_size << " " 
                     << duration.count() << "(us)" << std::endl;
            log_file.close();
        }
        
        // 사용된 명령 시간 정보 제거
        command_start_times.erase(it);
    } else {
        // 시작 시간을 찾을 수 없는 경우 (emplace 로그가 없는 경우)
        ensure_log_directory();
        std::ofstream log_file(get_log_file_path(), std::ios::app);
        
        if (log_file.is_open()) {
            log_file << get_current_time_string() << " [PopFront] " 
                     << command_id << " " << queue_size << " " 
                     << "Unknown start time" << std::endl;
            log_file.close();
        }
    }
}

/**
 * @brief 스레드가 뮤텍스를 획득할 때 로깅하는 함수
 * @param mutex_name 뮤텍스 이름 (예: "txq_mutex")
 * @param action 액션 타입 ("lock" 또는 "unlock")
 */
void chmy_thread_log(const std::string& mutex_name, const std::string& action) {
    std::lock_guard<std::mutex> lock(thread_stats_mutex);
    
    auto thread_id = std::this_thread::get_id();
    uint64_t thread_hash = std::hash<std::thread::id>{}(thread_id);
    
    auto current_time = std::chrono::steady_clock::now();
    
    if (action == "lock") {
        // 뮤텍스 획득 시
        mutex_lock_counts[mutex_name][thread_hash]++;
        mutex_last_lock_times[mutex_name][thread_hash] = current_time;
        
        // 로그 파일에 기록
        std::string log_file_path = "/home/rtcl-chmy/mavros_ws/src/mavros/libmavconn/log/thread_tracking_log.txt";
        std::filesystem::path file_path(log_file_path);
        std::filesystem::path dir_path = file_path.parent_path();
        
        if (!std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
        }
        
        std::ofstream log_file(log_file_path, std::ios::app);
        if (log_file.is_open()) {
            log_file << get_current_time_string() << " [LOCK] " 
                     << mutex_name << " ThreadID: " << thread_hash 
                     << " LockCount: " << mutex_lock_counts[mutex_name][thread_hash] << std::endl;
            log_file.close();
        }
        
    } else if (action == "unlock") {
        // 뮤텍스 해제 시
        auto it = mutex_last_lock_times[mutex_name].find(thread_hash);
        if (it != mutex_last_lock_times[mutex_name].end()) {
            auto lock_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                current_time - it->second);
            
            // 로그 파일에 기록
            std::string log_file_path = "/home/rtcl-chmy/mavros_ws/src/mavros/libmavconn/log/thread_tracking_log.txt";
            std::filesystem::path file_path(log_file_path);
            std::filesystem::path dir_path = file_path.parent_path();
            
            if (!std::filesystem::exists(dir_path)) {
                std::filesystem::create_directories(dir_path);
            }
            
            std::ofstream log_file(log_file_path, std::ios::app);
            if (log_file.is_open()) {
                log_file << get_current_time_string() << " [UNLOCK] " 
                         << mutex_name << " ThreadID: " << thread_hash 
                         << " LockDuration: " << lock_duration.count() << "μs" << std::endl;
                log_file.close();
            }
        }
    }
}

/**
 * @brief 스레드 추적 통계를 출력하는 함수
 */
void chmy_print_thread_stats() {
    std::lock_guard<std::mutex> lock(thread_stats_mutex);
    
    std::cout << "\n=== 스레드 뮤텍스 사용 통계 ===" << std::endl;
    
    for (const auto& mutex_pair : mutex_lock_counts) {
        std::cout << "\n뮤텍스: " << mutex_pair.first << std::endl;
        std::cout << "스레드별 락 횟수:" << std::endl;
        
        for (const auto& thread_pair : mutex_pair.second) {
            std::cout << "  ThreadID: " << thread_pair.first 
                      << " -> 횟수: " << thread_pair.second << std::endl;
        }
    }
    
    std::cout << "\n===============================" << std::endl;
}

} // namespace mavconn 