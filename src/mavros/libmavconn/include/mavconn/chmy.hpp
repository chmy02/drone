//
// libmavconn
// Copyright 2013,2014,2015,2016,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief CHMY utility functions header
 * @file chmy.hpp
 * @author CHMY
 *
 * @addtogroup mavconn
 * @{
 */

#pragma once
#ifndef MAVCONN__CHMY_HPP_
#define MAVCONN__CHMY_HPP_

#include <string>
#include <cstdint>

namespace mavconn
{

// ============================================================================
// 여기에 앞으로 추가하고 싶은 함수들의 선언을 정리해두세요
// ============================================================================

/**
 * @brief 예시 함수 - 필요에 따라 수정하거나 삭제하세요
 */
void example_function();

/**
 * @brief MAVLink 메시지 ID를 문자열로 변환하는 함수
 * @param msg_id MAVLink 메시지 ID
 * @return 메시지 이름 문자열
 */
std::string get_mavlink_message_name(uint32_t msg_id);

/**
 * @brief 시스템 상태를 확인하는 함수
 * @return 시스템 상태 문자열
 */
std::string check_system_status();

// ============================================================================
// 추가할 함수들의 선언을 여기에 계속 작성하세요
// ============================================================================

/**
 * @brief txq emplace 함수 실행 시점을 로깅하는 함수
 * @param command_id 명령 ID
 * @param queue_size 현재 큐 크기
 */
void chmy_emplace_log(uint32_t command_id, size_t queue_size);

/**
 * @brief txq popfront 함수 실행 시점을 로깅하는 함수
 * @param command_id 명령 ID
 * @param queue_size 현재 큐 크기
 */
void chmy_popfront_log(uint32_t command_id, size_t queue_size);

/**
 * @brief 스레드가 뮤텍스를 획득할 때 로깅하는 함수
 * @param mutex_name 뮤텍스 이름 (예: "txq_mutex")
 * @param action 액션 타입 ("lock" 또는 "unlock")
 */
void chmy_thread_log(const std::string& mutex_name, const std::string& action);

/**
 * @brief 스레드 추적 통계를 출력하는 함수
 */
void chmy_print_thread_stats();

} // namespace mavconn

#endif // MAVCONN__CHMY_HPP_ 