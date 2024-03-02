/** @file
 *    @brief MAVLink comm protocol testsuite generated from swarm.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef SWARM_TESTSUITE_H
#define SWARM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_swarm(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_swarm(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_led_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LED_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_led_control_t packet_in = {
        5,72,139,206,17
    };
    mavlink_led_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.r = packet_in.r;
        packet1.g = packet_in.g;
        packet1.b = packet_in.b;
        packet1.brightness = packet_in.brightness;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack(system_id, component_id, &msg , packet1.type , packet1.r , packet1.g , packet1.b , packet1.brightness );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.r , packet1.g , packet1.b , packet1.brightness );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_send(MAVLINK_COMM_1 , packet1.type , packet1.r , packet1.g , packet1.b , packet1.brightness );
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LED_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LED_CONTROL) != NULL);
#endif
}

static void mavlink_test_piksi_obs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PIKSI_OBS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_piksi_obs_t packet_in = {
        17235,17339,17443,151,218,{ 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192 }
    };
    mavlink_piksi_obs_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sender_id = packet_in.sender_id;
        packet1.msg_type = packet_in.msg_type;
        packet1.crc = packet_in.crc;
        packet1.seq = packet_in.seq;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*164);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_obs_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_piksi_obs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_obs_pack(system_id, component_id, &msg , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_obs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_obs_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_obs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_piksi_obs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_obs_send(MAVLINK_COMM_1 , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_obs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PIKSI_OBS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PIKSI_OBS) != NULL);
#endif
}

static void mavlink_test_piksi_base_pos_ecef(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_piksi_base_pos_ecef_t packet_in = {
        17235,17339,17443,151,218,{ 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52 }
    };
    mavlink_piksi_base_pos_ecef_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sender_id = packet_in.sender_id;
        packet1.msg_type = packet_in.msg_type;
        packet1.crc = packet_in.crc;
        packet1.seq = packet_in.seq;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*24);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_base_pos_ecef_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_piksi_base_pos_ecef_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_base_pos_ecef_pack(system_id, component_id, &msg , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_base_pos_ecef_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_base_pos_ecef_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_base_pos_ecef_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_piksi_base_pos_ecef_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_base_pos_ecef_send(MAVLINK_COMM_1 , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_base_pos_ecef_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PIKSI_BASE_POS_ECEF") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF) != NULL);
#endif
}

static void mavlink_test_target_offboard(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TARGET_OFFBOARD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_target_offboard_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,65,132,199,10,77,144,211,22
    };
    mavlink_target_offboard_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.pos_x = packet_in.pos_x;
        packet1.pos_y = packet_in.pos_y;
        packet1.pos_z = packet_in.pos_z;
        packet1.head = packet_in.head;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.led_type = packet_in.led_type;
        packet1.led_speed = packet_in.led_speed;
        packet1.led_r = packet_in.led_r;
        packet1.led_g = packet_in.led_g;
        packet1.led_b = packet_in.led_b;
        packet1.led_bright = packet_in.led_bright;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_offboard_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_target_offboard_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_offboard_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.target_system , packet1.target_component , packet1.led_type , packet1.led_speed , packet1.led_r , packet1.led_g , packet1.led_b , packet1.led_bright );
    mavlink_msg_target_offboard_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_offboard_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.target_system , packet1.target_component , packet1.led_type , packet1.led_speed , packet1.led_r , packet1.led_g , packet1.led_b , packet1.led_bright );
    mavlink_msg_target_offboard_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_target_offboard_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_offboard_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.target_system , packet1.target_component , packet1.led_type , packet1.led_speed , packet1.led_r , packet1.led_g , packet1.led_b , packet1.led_bright );
    mavlink_msg_target_offboard_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TARGET_OFFBOARD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TARGET_OFFBOARD) != NULL);
#endif
}

static void mavlink_test_monitoring(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MONITORING >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_monitoring_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,963498920,963499128,269.0,297.0,325.0,19731,27,94,161,228,39,106
    };
    mavlink_monitoring_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.tow = packet_in.tow;
        packet1.pos_x = packet_in.pos_x;
        packet1.pos_y = packet_in.pos_y;
        packet1.pos_z = packet_in.pos_z;
        packet1.head = packet_in.head;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.status1 = packet_in.status1;
        packet1.status2 = packet_in.status2;
        packet1.rtk_n = packet_in.rtk_n;
        packet1.rtk_e = packet_in.rtk_e;
        packet1.rtk_d = packet_in.rtk_d;
        packet1.swarm_id = packet_in.swarm_id;
        packet1.rtk_nbase = packet_in.rtk_nbase;
        packet1.rtk_nrover = packet_in.rtk_nrover;
        packet1.battery = packet_in.battery;
        packet1.r = packet_in.r;
        packet1.g = packet_in.g;
        packet1.b = packet_in.b;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MONITORING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MONITORING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_monitoring_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_pack(system_id, component_id, &msg , packet1.swarm_id , packet1.tow , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.roll , packet1.pitch , packet1.status1 , packet1.status2 , packet1.rtk_nbase , packet1.rtk_nrover , packet1.battery , packet1.r , packet1.g , packet1.b , packet1.rtk_n , packet1.rtk_e , packet1.rtk_d );
    mavlink_msg_monitoring_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.swarm_id , packet1.tow , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.roll , packet1.pitch , packet1.status1 , packet1.status2 , packet1.rtk_nbase , packet1.rtk_nrover , packet1.battery , packet1.r , packet1.g , packet1.b , packet1.rtk_n , packet1.rtk_e , packet1.rtk_d );
    mavlink_msg_monitoring_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_monitoring_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_send(MAVLINK_COMM_1 , packet1.swarm_id , packet1.tow , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.roll , packet1.pitch , packet1.status1 , packet1.status2 , packet1.rtk_nbase , packet1.rtk_nrover , packet1.battery , packet1.r , packet1.g , packet1.b , packet1.rtk_n , packet1.rtk_e , packet1.rtk_d );
    mavlink_msg_monitoring_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MONITORING") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MONITORING) != NULL);
#endif
}

static void mavlink_test_scenario_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCENARIO_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scenario_cmd_t packet_in = {
        17.0,45.0,73.0,963498088,53,120,{ 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218 }
    };
    mavlink_scenario_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.target_system = packet_in.target_system;
        packet1.cmd = packet_in.cmd;
        
        mav_array_memcpy(packet1.param5, packet_in.param5, sizeof(uint8_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scenario_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_pack(system_id, component_id, &msg , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.target_system , packet1.cmd , packet1.param5 );
    mavlink_msg_scenario_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.target_system , packet1.cmd , packet1.param5 );
    mavlink_msg_scenario_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scenario_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_send(MAVLINK_COMM_1 , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.target_system , packet1.cmd , packet1.param5 );
    mavlink_msg_scenario_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCENARIO_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCENARIO_CMD) != NULL);
#endif
}

static void mavlink_test_piksi_glo_biases(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PIKSI_GLO_BIASES >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_piksi_glo_biases_t packet_in = {
        17235,17339,17443,151,218,{ 29, 30, 31, 32, 33, 34, 35, 36, 37 }
    };
    mavlink_piksi_glo_biases_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sender_id = packet_in.sender_id;
        packet1.msg_type = packet_in.msg_type;
        packet1.crc = packet_in.crc;
        packet1.seq = packet_in.seq;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_glo_biases_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_piksi_glo_biases_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_glo_biases_pack(system_id, component_id, &msg , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_glo_biases_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_glo_biases_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_glo_biases_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_piksi_glo_biases_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_glo_biases_send(MAVLINK_COMM_1 , packet1.sender_id , packet1.msg_type , packet1.seq , packet1.crc , packet1.len , packet1.data );
    mavlink_msg_piksi_glo_biases_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PIKSI_GLO_BIASES") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PIKSI_GLO_BIASES) != NULL);
#endif
}

static void mavlink_test_piksi_pos_llh(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PIKSI_POS_LLH >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_piksi_pos_llh_t packet_in = {
        123.0,179.0,235.0,18483,18587,89,156
    };
    mavlink_piksi_pos_llh_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.height = packet_in.height;
        packet1.h_accuracy = packet_in.h_accuracy;
        packet1.v_accuracy = packet_in.v_accuracy;
        packet1.n_sats = packet_in.n_sats;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_pos_llh_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_piksi_pos_llh_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_pos_llh_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.height , packet1.h_accuracy , packet1.v_accuracy , packet1.n_sats , packet1.flags );
    mavlink_msg_piksi_pos_llh_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_pos_llh_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.height , packet1.h_accuracy , packet1.v_accuracy , packet1.n_sats , packet1.flags );
    mavlink_msg_piksi_pos_llh_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_piksi_pos_llh_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_piksi_pos_llh_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.height , packet1.h_accuracy , packet1.v_accuracy , packet1.n_sats , packet1.flags );
    mavlink_msg_piksi_pos_llh_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PIKSI_POS_LLH") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PIKSI_POS_LLH) != NULL);
#endif
}

static void mavlink_test_swarm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_led_control(system_id, component_id, last_msg);
    mavlink_test_piksi_obs(system_id, component_id, last_msg);
    mavlink_test_piksi_base_pos_ecef(system_id, component_id, last_msg);
    mavlink_test_target_offboard(system_id, component_id, last_msg);
    mavlink_test_monitoring(system_id, component_id, last_msg);
    mavlink_test_scenario_cmd(system_id, component_id, last_msg);
    mavlink_test_piksi_glo_biases(system_id, component_id, last_msg);
    mavlink_test_piksi_pos_llh(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SWARM_TESTSUITE_H
