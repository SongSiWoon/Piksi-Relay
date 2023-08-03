#pragma once
// MESSAGE PIKSI_BASE_POS_ECEF PACKING

#define MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF 182


typedef struct __mavlink_piksi_base_pos_ecef_t {
 uint16_t sender_id; /*<  base_pos_ecef sender id*/
 uint16_t msg_type; /*<  message type*/
 uint16_t crc; /*<  sbp crc*/
 uint8_t seq; /*<  message sequence*/
 uint8_t len; /*<  base_pos_ecef data length*/
 uint8_t data[24]; /*<  base_pos_ecef data*/
} mavlink_piksi_base_pos_ecef_t;

#define MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN 32
#define MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN 32
#define MAVLINK_MSG_ID_182_LEN 32
#define MAVLINK_MSG_ID_182_MIN_LEN 32

#define MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC 61
#define MAVLINK_MSG_ID_182_CRC 61

#define MAVLINK_MSG_PIKSI_BASE_POS_ECEF_FIELD_DATA_LEN 24

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIKSI_BASE_POS_ECEF { \
    182, \
    "PIKSI_BASE_POS_ECEF", \
    6, \
    {  { "sender_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_piksi_base_pos_ecef_t, sender_id) }, \
         { "msg_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_piksi_base_pos_ecef_t, msg_type) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_piksi_base_pos_ecef_t, seq) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_piksi_base_pos_ecef_t, crc) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_piksi_base_pos_ecef_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 24, 8, offsetof(mavlink_piksi_base_pos_ecef_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIKSI_BASE_POS_ECEF { \
    "PIKSI_BASE_POS_ECEF", \
    6, \
    {  { "sender_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_piksi_base_pos_ecef_t, sender_id) }, \
         { "msg_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_piksi_base_pos_ecef_t, msg_type) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_piksi_base_pos_ecef_t, seq) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_piksi_base_pos_ecef_t, crc) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_piksi_base_pos_ecef_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 24, 8, offsetof(mavlink_piksi_base_pos_ecef_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a piksi_base_pos_ecef message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sender_id  base_pos_ecef sender id
 * @param msg_type  message type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  base_pos_ecef data length
 * @param data  base_pos_ecef data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN);
#else
    mavlink_piksi_base_pos_ecef_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC);
}

/**
 * @brief Pack a piksi_base_pos_ecef message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sender_id  base_pos_ecef sender id
 * @param msg_type  message type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  base_pos_ecef data length
 * @param data  base_pos_ecef data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t sender_id,uint16_t msg_type,uint8_t seq,uint16_t crc,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN);
#else
    mavlink_piksi_base_pos_ecef_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC);
}

/**
 * @brief Encode a piksi_base_pos_ecef struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param piksi_base_pos_ecef C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_piksi_base_pos_ecef_t* piksi_base_pos_ecef)
{
    return mavlink_msg_piksi_base_pos_ecef_pack(system_id, component_id, msg, piksi_base_pos_ecef->sender_id, piksi_base_pos_ecef->msg_type, piksi_base_pos_ecef->seq, piksi_base_pos_ecef->crc, piksi_base_pos_ecef->len, piksi_base_pos_ecef->data);
}

/**
 * @brief Encode a piksi_base_pos_ecef struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param piksi_base_pos_ecef C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_piksi_base_pos_ecef_t* piksi_base_pos_ecef)
{
    return mavlink_msg_piksi_base_pos_ecef_pack_chan(system_id, component_id, chan, msg, piksi_base_pos_ecef->sender_id, piksi_base_pos_ecef->msg_type, piksi_base_pos_ecef->seq, piksi_base_pos_ecef->crc, piksi_base_pos_ecef->len, piksi_base_pos_ecef->data);
}

/**
 * @brief Send a piksi_base_pos_ecef message
 * @param chan MAVLink channel to send the message
 *
 * @param sender_id  base_pos_ecef sender id
 * @param msg_type  message type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  base_pos_ecef data length
 * @param data  base_pos_ecef data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_piksi_base_pos_ecef_send(mavlink_channel_t chan, uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF, buf, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC);
#else
    mavlink_piksi_base_pos_ecef_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF, (const char *)&packet, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC);
#endif
}

/**
 * @brief Send a piksi_base_pos_ecef message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_piksi_base_pos_ecef_send_struct(mavlink_channel_t chan, const mavlink_piksi_base_pos_ecef_t* piksi_base_pos_ecef)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_piksi_base_pos_ecef_send(chan, piksi_base_pos_ecef->sender_id, piksi_base_pos_ecef->msg_type, piksi_base_pos_ecef->seq, piksi_base_pos_ecef->crc, piksi_base_pos_ecef->len, piksi_base_pos_ecef->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF, (const char *)piksi_base_pos_ecef, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_piksi_base_pos_ecef_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF, buf, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC);
#else
    mavlink_piksi_base_pos_ecef_t *packet = (mavlink_piksi_base_pos_ecef_t *)msgbuf;
    packet->sender_id = sender_id;
    packet->msg_type = msg_type;
    packet->crc = crc;
    packet->seq = seq;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF, (const char *)packet, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_MIN_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_CRC);
#endif
}
#endif

#endif

// MESSAGE PIKSI_BASE_POS_ECEF UNPACKING


/**
 * @brief Get field sender_id from piksi_base_pos_ecef message
 *
 * @return  base_pos_ecef sender id
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_get_sender_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field msg_type from piksi_base_pos_ecef message
 *
 * @return  message type
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_get_msg_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field seq from piksi_base_pos_ecef message
 *
 * @return  message sequence
 */
static inline uint8_t mavlink_msg_piksi_base_pos_ecef_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field crc from piksi_base_pos_ecef message
 *
 * @return  sbp crc
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_get_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field len from piksi_base_pos_ecef message
 *
 * @return  base_pos_ecef data length
 */
static inline uint8_t mavlink_msg_piksi_base_pos_ecef_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field data from piksi_base_pos_ecef message
 *
 * @return  base_pos_ecef data
 */
static inline uint16_t mavlink_msg_piksi_base_pos_ecef_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 24,  8);
}

/**
 * @brief Decode a piksi_base_pos_ecef message into a struct
 *
 * @param msg The message to decode
 * @param piksi_base_pos_ecef C-struct to decode the message contents into
 */
static inline void mavlink_msg_piksi_base_pos_ecef_decode(const mavlink_message_t* msg, mavlink_piksi_base_pos_ecef_t* piksi_base_pos_ecef)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    piksi_base_pos_ecef->sender_id = mavlink_msg_piksi_base_pos_ecef_get_sender_id(msg);
    piksi_base_pos_ecef->msg_type = mavlink_msg_piksi_base_pos_ecef_get_msg_type(msg);
    piksi_base_pos_ecef->crc = mavlink_msg_piksi_base_pos_ecef_get_crc(msg);
    piksi_base_pos_ecef->seq = mavlink_msg_piksi_base_pos_ecef_get_seq(msg);
    piksi_base_pos_ecef->len = mavlink_msg_piksi_base_pos_ecef_get_len(msg);
    mavlink_msg_piksi_base_pos_ecef_get_data(msg, piksi_base_pos_ecef->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN? msg->len : MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN;
        memset(piksi_base_pos_ecef, 0, MAVLINK_MSG_ID_PIKSI_BASE_POS_ECEF_LEN);
    memcpy(piksi_base_pos_ecef, _MAV_PAYLOAD(msg), len);
#endif
}
