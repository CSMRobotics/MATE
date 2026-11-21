/** @file
 *  @brief MAVLink comm testsuite protocol generated from rov_interfaces.xml
 *  @see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "rov_interfaces.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(rov_interfaces, SUB_CONTROLS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::rov_interfaces::msg::SUB_CONTROLS packet_in{};
    packet_in.velocity_x = 17.0;
    packet_in.velocity_y = 45.0;
    packet_in.velocity_z = 73.0;
    packet_in.omega_x = 101.0;
    packet_in.omega_y = 129.0;
    packet_in.omega_z = 157.0;
    packet_in.mode_switch = 77;
    packet_in.estop = 144;

    mavlink::rov_interfaces::msg::SUB_CONTROLS packet1{};
    mavlink::rov_interfaces::msg::SUB_CONTROLS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.velocity_x, packet2.velocity_x);
    EXPECT_EQ(packet1.velocity_y, packet2.velocity_y);
    EXPECT_EQ(packet1.velocity_z, packet2.velocity_z);
    EXPECT_EQ(packet1.omega_x, packet2.omega_x);
    EXPECT_EQ(packet1.omega_y, packet2.omega_y);
    EXPECT_EQ(packet1.omega_z, packet2.omega_z);
    EXPECT_EQ(packet1.mode_switch, packet2.mode_switch);
    EXPECT_EQ(packet1.estop, packet2.estop);
}

#ifdef TEST_INTEROP
TEST(rov_interfaces_interop, SUB_CONTROLS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_sub_controls_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 77, 144
    };

    mavlink::rov_interfaces::msg::SUB_CONTROLS packet_in{};
    packet_in.velocity_x = 17.0;
    packet_in.velocity_y = 45.0;
    packet_in.velocity_z = 73.0;
    packet_in.omega_x = 101.0;
    packet_in.omega_y = 129.0;
    packet_in.omega_z = 157.0;
    packet_in.mode_switch = 77;
    packet_in.estop = 144;

    mavlink::rov_interfaces::msg::SUB_CONTROLS packet2{};

    mavlink_msg_sub_controls_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.velocity_x, packet2.velocity_x);
    EXPECT_EQ(packet_in.velocity_y, packet2.velocity_y);
    EXPECT_EQ(packet_in.velocity_z, packet2.velocity_z);
    EXPECT_EQ(packet_in.omega_x, packet2.omega_x);
    EXPECT_EQ(packet_in.omega_y, packet2.omega_y);
    EXPECT_EQ(packet_in.omega_z, packet2.omega_z);
    EXPECT_EQ(packet_in.mode_switch, packet2.mode_switch);
    EXPECT_EQ(packet_in.estop, packet2.estop);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
