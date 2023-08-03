/** @file
 *  @brief MAVLink comm protocol generated from swarm.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_SWARM_H
#define MAVLINK_SWARM_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_SWARM.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_SWARM_XML_HASH -8058789401026445328

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{180, 5, 5, 5, 0, 0, 0}, {181, 173, 172, 172, 0, 0, 0}, {182, 61, 32, 32, 0, 0, 0}, {183, 207, 28, 28, 3, 20, 21}, {184, 96, 54, 54, 0, 0, 0}, {185, 180, 50, 50, 1, 16, 0}, {186, 188, 17, 17, 0, 0, 0}, {187, 52, 30, 30, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_SWARM

// ENUM DEFINITIONS


/** @brief Enumeration of distance sensor types */
#ifndef HAVE_ENUM_SCENARIO_CMD_ENUM
#define HAVE_ENUM_SCENARIO_CMD_ENUM
typedef enum SCENARIO_CMD_ENUM
{
   SCENARIO_CMD_SET_START_TIME=0, /* set start time for scenario  (param1: start_time) | */
   SCENARIO_CMD_SET_RELATIVE_POS=1, /* set relative position (param1 : x, param2: y) | */
   SCENARIO_CMD_SELECT_SCENARIO_FILE=2, /* set scenario file name (param5 : file name) | */
   SCENARIO_CMD_STOP_SCENARIO=3, /* stop scenario | */
   SCENARIO_CMD_EMERGENCY_LAND=4, /* land | */
   SCENARIO_CMD_SET_CONFIGS=5, /* set vairous configurations param1: base vehicle pos x, param2: base vehicle pos y, param5: scenario file name | */
   SCENARIO_CMD_RESET_CONFIGS=6, /* reset configurations | */
   SCENARIO_CMD_ENUM_ENUM_END=7, /*  | */
} SCENARIO_CMD_ENUM;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_led_control.h"
#include "./mavlink_msg_piksi_obs.h"
#include "./mavlink_msg_piksi_base_pos_ecef.h"
#include "./mavlink_msg_target_offboard.h"
#include "./mavlink_msg_monitoring.h"
#include "./mavlink_msg_scenario_cmd.h"
#include "./mavlink_msg_piksi_glo_biases.h"
#include "./mavlink_msg_piksi_pos_llh.h"

// base include



#if MAVLINK_SWARM_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_LED_CONTROL, MAVLINK_MESSAGE_INFO_PIKSI_OBS, MAVLINK_MESSAGE_INFO_PIKSI_BASE_POS_ECEF, MAVLINK_MESSAGE_INFO_TARGET_OFFBOARD, MAVLINK_MESSAGE_INFO_MONITORING, MAVLINK_MESSAGE_INFO_SCENARIO_CMD, MAVLINK_MESSAGE_INFO_PIKSI_GLO_BIASES, MAVLINK_MESSAGE_INFO_PIKSI_POS_LLH}
# define MAVLINK_MESSAGE_NAMES {{ "LED_CONTROL", 180 }, { "MONITORING", 184 }, { "PIKSI_BASE_POS_ECEF", 182 }, { "PIKSI_GLO_BIASES", 186 }, { "PIKSI_OBS", 181 }, { "PIKSI_POS_LLH", 187 }, { "SCENARIO_CMD", 185 }, { "TARGET_OFFBOARD", 183 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_SWARM_H
