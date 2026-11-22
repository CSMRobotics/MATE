#pragma once
// MESSAGE SUB_CONTROLS PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_SUB_CONTROLS 200


typedef struct __mavlink_sub_controls_t {
 float velocity_x; /*<  
                Forward velocity (+Forward)
            */
 float velocity_y; /*<  
                Lateral velocity (+Right)
            */
 float velocity_z; /*<  
                Vertical velocity (+Down)
            */
 float omega_x; /*<  
                Roll rate (+Clockwise looking down x axis, from back to front)
            */
 float omega_y; /*<  
                Pitch rate (+Pull up)
            */
 float omega_z; /*<  
                Yaw rate (+Turn right)
            */
 uint8_t mode_switch; /*<  
                Mode switch (0 = disabled, 1 = enabled)
            */
 uint8_t estop; /*<  
                Emergency stop (0 = inactive, 1 = active)
            */
} mavlink_sub_controls_t;

#define MAVLINK_MSG_ID_SUB_CONTROLS_LEN 26
#define MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN 26
#define MAVLINK_MSG_ID_200_LEN 26
#define MAVLINK_MSG_ID_200_MIN_LEN 26

#define MAVLINK_MSG_ID_SUB_CONTROLS_CRC 17
#define MAVLINK_MSG_ID_200_CRC 17



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SUB_CONTROLS { \
    200, \
    "SUB_CONTROLS", \
    8, \
    {  { "velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sub_controls_t, velocity_x) }, \
         { "velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sub_controls_t, velocity_y) }, \
         { "velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sub_controls_t, velocity_z) }, \
         { "omega_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sub_controls_t, omega_x) }, \
         { "omega_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sub_controls_t, omega_y) }, \
         { "omega_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sub_controls_t, omega_z) }, \
         { "mode_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sub_controls_t, mode_switch) }, \
         { "estop", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_sub_controls_t, estop) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SUB_CONTROLS { \
    "SUB_CONTROLS", \
    8, \
    {  { "velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sub_controls_t, velocity_x) }, \
         { "velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sub_controls_t, velocity_y) }, \
         { "velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sub_controls_t, velocity_z) }, \
         { "omega_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sub_controls_t, omega_x) }, \
         { "omega_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sub_controls_t, omega_y) }, \
         { "omega_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sub_controls_t, omega_z) }, \
         { "mode_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sub_controls_t, mode_switch) }, \
         { "estop", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_sub_controls_t, estop) }, \
         } \
}
#endif

/**
 * @brief Pack a sub_controls message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param velocity_x  
                Forward velocity (+Forward)
            
 * @param velocity_y  
                Lateral velocity (+Right)
            
 * @param velocity_z  
                Vertical velocity (+Down)
            
 * @param omega_x  
                Roll rate (+Clockwise looking down x axis, from back to front)
            
 * @param omega_y  
                Pitch rate (+Pull up)
            
 * @param omega_z  
                Yaw rate (+Turn right)
            
 * @param mode_switch  
                Mode switch (0 = disabled, 1 = enabled)
            
 * @param estop  
                Emergency stop (0 = inactive, 1 = active)
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sub_controls_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float velocity_x, float velocity_y, float velocity_z, float omega_x, float omega_y, float omega_z, uint8_t mode_switch, uint8_t estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SUB_CONTROLS_LEN];
    _mav_put_float(buf, 0, velocity_x);
    _mav_put_float(buf, 4, velocity_y);
    _mav_put_float(buf, 8, velocity_z);
    _mav_put_float(buf, 12, omega_x);
    _mav_put_float(buf, 16, omega_y);
    _mav_put_float(buf, 20, omega_z);
    _mav_put_uint8_t(buf, 24, mode_switch);
    _mav_put_uint8_t(buf, 25, estop);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
#else
    mavlink_sub_controls_t packet;
    packet.velocity_x = velocity_x;
    packet.velocity_y = velocity_y;
    packet.velocity_z = velocity_z;
    packet.omega_x = omega_x;
    packet.omega_y = omega_y;
    packet.omega_z = omega_z;
    packet.mode_switch = mode_switch;
    packet.estop = estop;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SUB_CONTROLS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
}

/**
 * @brief Pack a sub_controls message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param velocity_x  
                Forward velocity (+Forward)
            
 * @param velocity_y  
                Lateral velocity (+Right)
            
 * @param velocity_z  
                Vertical velocity (+Down)
            
 * @param omega_x  
                Roll rate (+Clockwise looking down x axis, from back to front)
            
 * @param omega_y  
                Pitch rate (+Pull up)
            
 * @param omega_z  
                Yaw rate (+Turn right)
            
 * @param mode_switch  
                Mode switch (0 = disabled, 1 = enabled)
            
 * @param estop  
                Emergency stop (0 = inactive, 1 = active)
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sub_controls_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float velocity_x, float velocity_y, float velocity_z, float omega_x, float omega_y, float omega_z, uint8_t mode_switch, uint8_t estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SUB_CONTROLS_LEN];
    _mav_put_float(buf, 0, velocity_x);
    _mav_put_float(buf, 4, velocity_y);
    _mav_put_float(buf, 8, velocity_z);
    _mav_put_float(buf, 12, omega_x);
    _mav_put_float(buf, 16, omega_y);
    _mav_put_float(buf, 20, omega_z);
    _mav_put_uint8_t(buf, 24, mode_switch);
    _mav_put_uint8_t(buf, 25, estop);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
#else
    mavlink_sub_controls_t packet;
    packet.velocity_x = velocity_x;
    packet.velocity_y = velocity_y;
    packet.velocity_z = velocity_z;
    packet.omega_x = omega_x;
    packet.omega_y = omega_y;
    packet.omega_z = omega_z;
    packet.mode_switch = mode_switch;
    packet.estop = estop;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SUB_CONTROLS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
#endif
}

/**
 * @brief Pack a sub_controls message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param velocity_x  
                Forward velocity (+Forward)
            
 * @param velocity_y  
                Lateral velocity (+Right)
            
 * @param velocity_z  
                Vertical velocity (+Down)
            
 * @param omega_x  
                Roll rate (+Clockwise looking down x axis, from back to front)
            
 * @param omega_y  
                Pitch rate (+Pull up)
            
 * @param omega_z  
                Yaw rate (+Turn right)
            
 * @param mode_switch  
                Mode switch (0 = disabled, 1 = enabled)
            
 * @param estop  
                Emergency stop (0 = inactive, 1 = active)
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sub_controls_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float velocity_x,float velocity_y,float velocity_z,float omega_x,float omega_y,float omega_z,uint8_t mode_switch,uint8_t estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SUB_CONTROLS_LEN];
    _mav_put_float(buf, 0, velocity_x);
    _mav_put_float(buf, 4, velocity_y);
    _mav_put_float(buf, 8, velocity_z);
    _mav_put_float(buf, 12, omega_x);
    _mav_put_float(buf, 16, omega_y);
    _mav_put_float(buf, 20, omega_z);
    _mav_put_uint8_t(buf, 24, mode_switch);
    _mav_put_uint8_t(buf, 25, estop);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
#else
    mavlink_sub_controls_t packet;
    packet.velocity_x = velocity_x;
    packet.velocity_y = velocity_y;
    packet.velocity_z = velocity_z;
    packet.omega_x = omega_x;
    packet.omega_y = omega_y;
    packet.omega_z = omega_z;
    packet.mode_switch = mode_switch;
    packet.estop = estop;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SUB_CONTROLS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
}

/**
 * @brief Encode a sub_controls struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sub_controls C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sub_controls_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sub_controls_t* sub_controls)
{
    return mavlink_msg_sub_controls_pack(system_id, component_id, msg, sub_controls->velocity_x, sub_controls->velocity_y, sub_controls->velocity_z, sub_controls->omega_x, sub_controls->omega_y, sub_controls->omega_z, sub_controls->mode_switch, sub_controls->estop);
}

/**
 * @brief Encode a sub_controls struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sub_controls C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sub_controls_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sub_controls_t* sub_controls)
{
    return mavlink_msg_sub_controls_pack_chan(system_id, component_id, chan, msg, sub_controls->velocity_x, sub_controls->velocity_y, sub_controls->velocity_z, sub_controls->omega_x, sub_controls->omega_y, sub_controls->omega_z, sub_controls->mode_switch, sub_controls->estop);
}

/**
 * @brief Encode a sub_controls struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sub_controls C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sub_controls_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sub_controls_t* sub_controls)
{
    return mavlink_msg_sub_controls_pack_status(system_id, component_id, _status, msg,  sub_controls->velocity_x, sub_controls->velocity_y, sub_controls->velocity_z, sub_controls->omega_x, sub_controls->omega_y, sub_controls->omega_z, sub_controls->mode_switch, sub_controls->estop);
}

/**
 * @brief Send a sub_controls message
 * @param chan MAVLink channel to send the message
 *
 * @param velocity_x  
                Forward velocity (+Forward)
            
 * @param velocity_y  
                Lateral velocity (+Right)
            
 * @param velocity_z  
                Vertical velocity (+Down)
            
 * @param omega_x  
                Roll rate (+Clockwise looking down x axis, from back to front)
            
 * @param omega_y  
                Pitch rate (+Pull up)
            
 * @param omega_z  
                Yaw rate (+Turn right)
            
 * @param mode_switch  
                Mode switch (0 = disabled, 1 = enabled)
            
 * @param estop  
                Emergency stop (0 = inactive, 1 = active)
            
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sub_controls_send(mavlink_channel_t chan, float velocity_x, float velocity_y, float velocity_z, float omega_x, float omega_y, float omega_z, uint8_t mode_switch, uint8_t estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SUB_CONTROLS_LEN];
    _mav_put_float(buf, 0, velocity_x);
    _mav_put_float(buf, 4, velocity_y);
    _mav_put_float(buf, 8, velocity_z);
    _mav_put_float(buf, 12, omega_x);
    _mav_put_float(buf, 16, omega_y);
    _mav_put_float(buf, 20, omega_z);
    _mav_put_uint8_t(buf, 24, mode_switch);
    _mav_put_uint8_t(buf, 25, estop);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SUB_CONTROLS, buf, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
#else
    mavlink_sub_controls_t packet;
    packet.velocity_x = velocity_x;
    packet.velocity_y = velocity_y;
    packet.velocity_z = velocity_z;
    packet.omega_x = omega_x;
    packet.omega_y = omega_y;
    packet.omega_z = omega_z;
    packet.mode_switch = mode_switch;
    packet.estop = estop;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SUB_CONTROLS, (const char *)&packet, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
#endif
}

/**
 * @brief Send a sub_controls message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sub_controls_send_struct(mavlink_channel_t chan, const mavlink_sub_controls_t* sub_controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sub_controls_send(chan, sub_controls->velocity_x, sub_controls->velocity_y, sub_controls->velocity_z, sub_controls->omega_x, sub_controls->omega_y, sub_controls->omega_z, sub_controls->mode_switch, sub_controls->estop);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SUB_CONTROLS, (const char *)sub_controls, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SUB_CONTROLS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sub_controls_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float velocity_x, float velocity_y, float velocity_z, float omega_x, float omega_y, float omega_z, uint8_t mode_switch, uint8_t estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, velocity_x);
    _mav_put_float(buf, 4, velocity_y);
    _mav_put_float(buf, 8, velocity_z);
    _mav_put_float(buf, 12, omega_x);
    _mav_put_float(buf, 16, omega_y);
    _mav_put_float(buf, 20, omega_z);
    _mav_put_uint8_t(buf, 24, mode_switch);
    _mav_put_uint8_t(buf, 25, estop);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SUB_CONTROLS, buf, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
#else
    mavlink_sub_controls_t *packet = (mavlink_sub_controls_t *)msgbuf;
    packet->velocity_x = velocity_x;
    packet->velocity_y = velocity_y;
    packet->velocity_z = velocity_z;
    packet->omega_x = omega_x;
    packet->omega_y = omega_y;
    packet->omega_z = omega_z;
    packet->mode_switch = mode_switch;
    packet->estop = estop;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SUB_CONTROLS, (const char *)packet, MAVLINK_MSG_ID_SUB_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_LEN, MAVLINK_MSG_ID_SUB_CONTROLS_CRC);
#endif
}
#endif

#endif

// MESSAGE SUB_CONTROLS UNPACKING


/**
 * @brief Get field velocity_x from sub_controls message
 *
 * @return  
                Forward velocity (+Forward)
            
 */
static inline float mavlink_msg_sub_controls_get_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field velocity_y from sub_controls message
 *
 * @return  
                Lateral velocity (+Right)
            
 */
static inline float mavlink_msg_sub_controls_get_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field velocity_z from sub_controls message
 *
 * @return  
                Vertical velocity (+Down)
            
 */
static inline float mavlink_msg_sub_controls_get_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field omega_x from sub_controls message
 *
 * @return  
                Roll rate (+Clockwise looking down x axis, from back to front)
            
 */
static inline float mavlink_msg_sub_controls_get_omega_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field omega_y from sub_controls message
 *
 * @return  
                Pitch rate (+Pull up)
            
 */
static inline float mavlink_msg_sub_controls_get_omega_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field omega_z from sub_controls message
 *
 * @return  
                Yaw rate (+Turn right)
            
 */
static inline float mavlink_msg_sub_controls_get_omega_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field mode_switch from sub_controls message
 *
 * @return  
                Mode switch (0 = disabled, 1 = enabled)
            
 */
static inline uint8_t mavlink_msg_sub_controls_get_mode_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field estop from sub_controls message
 *
 * @return  
                Emergency stop (0 = inactive, 1 = active)
            
 */
static inline uint8_t mavlink_msg_sub_controls_get_estop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Decode a sub_controls message into a struct
 *
 * @param msg The message to decode
 * @param sub_controls C-struct to decode the message contents into
 */
static inline void mavlink_msg_sub_controls_decode(const mavlink_message_t* msg, mavlink_sub_controls_t* sub_controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sub_controls->velocity_x = mavlink_msg_sub_controls_get_velocity_x(msg);
    sub_controls->velocity_y = mavlink_msg_sub_controls_get_velocity_y(msg);
    sub_controls->velocity_z = mavlink_msg_sub_controls_get_velocity_z(msg);
    sub_controls->omega_x = mavlink_msg_sub_controls_get_omega_x(msg);
    sub_controls->omega_y = mavlink_msg_sub_controls_get_omega_y(msg);
    sub_controls->omega_z = mavlink_msg_sub_controls_get_omega_z(msg);
    sub_controls->mode_switch = mavlink_msg_sub_controls_get_mode_switch(msg);
    sub_controls->estop = mavlink_msg_sub_controls_get_estop(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SUB_CONTROLS_LEN? msg->len : MAVLINK_MSG_ID_SUB_CONTROLS_LEN;
        memset(sub_controls, 0, MAVLINK_MSG_ID_SUB_CONTROLS_LEN);
    memcpy(sub_controls, _MAV_PAYLOAD(msg), len);
#endif
}
