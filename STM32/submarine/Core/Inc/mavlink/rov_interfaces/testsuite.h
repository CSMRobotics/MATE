/** @file
 *    @brief MAVLink comm protocol testsuite generated from rov_interfaces.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ROV_INTERFACES_TESTSUITE_H
#define ROV_INTERFACES_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_rov_interfaces(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_rov_interfaces(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_sub_controls(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SUB_CONTROLS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sub_controls_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77,144
    };
    mavlink_sub_controls_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.velocity_x = packet_in.velocity_x;
        packet1.velocity_y = packet_in.velocity_y;
        packet1.velocity_z = packet_in.velocity_z;
        packet1.omega_x = packet_in.omega_x;
        packet1.omega_y = packet_in.omega_y;
        packet1.omega_z = packet_in.omega_z;
        packet1.mode_switch = packet_in.mode_switch;
        packet1.estop = packet_in.estop;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sub_controls_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sub_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sub_controls_pack(system_id, component_id, &msg , packet1.velocity_x , packet1.velocity_y , packet1.velocity_z , packet1.omega_x , packet1.omega_y , packet1.omega_z , packet1.mode_switch , packet1.estop );
    mavlink_msg_sub_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sub_controls_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.velocity_x , packet1.velocity_y , packet1.velocity_z , packet1.omega_x , packet1.omega_y , packet1.omega_z , packet1.mode_switch , packet1.estop );
    mavlink_msg_sub_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sub_controls_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sub_controls_send(MAVLINK_COMM_1 , packet1.velocity_x , packet1.velocity_y , packet1.velocity_z , packet1.omega_x , packet1.omega_y , packet1.omega_z , packet1.mode_switch , packet1.estop );
    mavlink_msg_sub_controls_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SUB_CONTROLS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SUB_CONTROLS) != NULL);
#endif
}

static void mavlink_test_rov_interfaces(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_sub_controls(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ROV_INTERFACES_TESTSUITE_H
