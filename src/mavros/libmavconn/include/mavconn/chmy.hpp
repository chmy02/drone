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
 * @file chmy.hpp
 * @author chmy
 *
 * @addtogroup mavconn
 * @{
 */

#pragma once
#ifndef MAVCONN__CHMY_HPP_
#define MAVCONN__CHMY_HPP_

#include <cstdint>

namespace mavconn
{

/**
 * @brief Log emplace operation with message ID
 * @param msgid Message ID to log
 */
void chmy_emplace_log(uint32_t msgid);

/**
 * @brief Log pop operation
 */
void chmy_pop_log();

}  // namespace mavconn

#endif  // MAVCONN__CHMY_HPP_
