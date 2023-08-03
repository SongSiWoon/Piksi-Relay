#pragma once
// MESSAGE PIKSI_OBS PACKING

#define MAVLINK_MSG_ID_PIKSI_OBS 181


typedef struct __mavlink_piksi_obs_t {
 uint16_t sender_id; /*<  observation sender id*/
 uint16_t msg_type; /*<  message type*/
 uint16_t crc; /*<  sbp crc*/
 uint8_t seq; /*<  message sequence*/
 uint8_t len; /*<  observation data length*/
 uint8_t data[164]; /*<  observation data*/
} mavlink_piksi_obs_t;

#define MAVLINK_MSG_ID_PIKSI_OBS_LEN 172
#define MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN 172
#define MAVLINK_MSG_ID_181_LEN 172
#define MAVLINK_MSG_ID_181_MIN_LEN 172

#define MAVLINK_MSG_ID_PIKSI_OBS_CRC 173
#define MAVLINK_MSG_ID_181_CRC 173

#define MAVLINK_MSG_PIKSI_OBS_FIELD_DATA_LEN 164

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIKSI_OBS { \
    181, \
    "PIKSI_OBS", \
    6, \
    {  { "sender_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_piksi_obs_t, sender_id) }, \
         { "msg_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_piksi_obs_t, msg_type) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_piksi_obs_t, seq) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_piksi_obs_t, crc) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_piksi_obs_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 164, 8, offsetof(mavlink_piksi_obs_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIKSI_OBS { \
    "PIKSI_OBS", \
    6, \
    {  { "sender_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_piksi_obs_t, sender_id) }, \
         { "msg_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_piksi_obs_t, msg_type) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_piksi_obs_t, seq) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_piksi_obs_t, crc) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_piksi_obs_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 164, 8, offsetof(mavlink_piksi_obs_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a piksi_obs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sender_id  observation sender id
 * @param msg_type  message type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  observation data length
 * @param data  observation data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_obs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_OBS_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 164);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_OBS_LEN);
#else
    mavlink_piksi_obs_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*164);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_OBS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_OBS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN, MAVLINK_MSG_ID_PIKSI_OBS_LEN, MAVLINK_MSG_ID_PIKSI_OBS_CRC);
}

/**
 * @brief Pack a piksi_obs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sender_id  observation sender id
 * @param msg_type  message type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  observation data length
 * @param data  observation data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_obs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t sender_id,uint16_t msg_type,uint8_t seq,uint16_t crc,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_OBS_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 164);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_OBS_LEN);
#else
    mavlink_piksi_obs_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*164);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_OBS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_OBS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN, MAVLINK_MSG_ID_PIKSI_OBS_LEN, MAVLINK_MSG_ID_PIKSI_OBS_CRC);
}

/**
 * @brief Encode a piksi_obs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param piksi_obs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_obs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_piksi_obs_t* piksi_obs)
{
    return mavlink_msg_piksi_obs_pack(system_id, component_id, msg, piksi_obs->sender_id, piksi_obs->msg_type, piksi_obs->seq, piksi_obs->crc, piksi_obs->len, piksi_obs->data);
}

/**
 * @brief Encode a piksi_obs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param piksi_obs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_obs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_piksi_obs_t* piksi_obs)
{
    return mavlink_msg_piksi_obs_pack_chan(system_id, component_id, chan, msg, piksi_obs->sender_id, piksi_obs->msg_type, piksi_obs->seq, piksi_obs->crc, piksi_obs->len, piksi_obs->data);
}

/**
 * @brief Send a piksi_obs message
 * @param chan MAVLink channel to send the message
 *
 * @param sender_id  observation sender id
 * @param msg_type  message type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  observation data length
 * @param data  observation data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_piksi_obs_send(mavlink_channel_t chan, uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_OBS_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 164);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_OBS, buf, MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN, MAVLINK_MSG_ID_PIKSI_OBS_LEN, MAVLINK_MSG_ID_PIKSI_OBS_CRC);
#else
    mavlink_piksi_obs_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*164);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_OBS, (const char *)&packet, MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN, MAVLINK_MSG_ID_PIKSI_OBS_LEN, MAVLINK_MSG_ID_PIKSI_OBS_CRC);
#endif
}

/**
 * @brief Send a piksi_obs message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_piksi_obs_send_struct(mavlink_channel_t chan, const mavlink_piksi_obs_t* piksi_obs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_piksi_obs_send(chan, piksi_obs->sender_id, piksi_obs->msg_type, piksi_obs->seq, piksi_obs->crc, piksi_obs->len, piksi_obs->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_OBS, (const char *)piksi_obs, MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN, MAVLINK_MSG_ID_PIKSI_OBS_LEN, MAVLINK_MSG_ID_PIKSI_OBS_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIKSI_OBS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_piksi_obs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 164);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_OBS, buf, MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN, MAVLINK_MSG_ID_PIKSI_OBS_LEN, MAVLINK_MSG_ID_PIKSI_OBS_CRC);
#else
    mavlink_piksi_obs_t *packet = (mavlink_piksi_obs_t *)msgbuf;
    packet->sender_id = sender_id;
    packet->msg_type = msg_type;
    packet->crc = crc;
    packet->seq = seq;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*164);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_OBS, (const char *)packet, MAVLINK_MSG_ID_PIKSI_OBS_MIN_LEN, MAVLINK_MSG_ID_PIKSI_OBS_LEN, MAVLINK_MSG_ID_PIKSI_OBS_CRC);
#endif
}
#endif

#endif

// MESSAGE PIKSI_OBS UNPACKING


/**
 * @brief Get field sender_id from piksi_obs message
 *
 * @return  observation sender id
 */
static inline uint16_t mavlink_msg_piksi_obs_get_sender_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field msg_type from piksi_obs message
 *
 * @return  message type
 */
static inline uint16_t mavlink_msg_piksi_obs_get_msg_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field seq from piksi_obs message
 *
 * @return  message sequence
 */
static inline uint8_t mavlink_msg_piksi_obs_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field crc from piksi_obs message
 *
 * @return  sbp crc
 */
static inline uint16_t mavlink_msg_piksi_obs_get_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field len from piksi_obs message
 *
 * @return  observation data length
 */
static inline uint8_t mavlink_msg_piksi_obs_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field data from piksi_obs message
 *
 * @return  observation data
 */
static inline uint16_t mavlink_msg_piksi_obs_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 164,  8);
}

/**
 * @brief Decode a piksi_obs message into a struct
 *
 * @param msg The message to decode
 * @param piksi_obs C-struct to decode the message contents into
 */
static inline void mavlink_msg_piksi_obs_decode(const mavlink_message_t* msg, mavlink_piksi_obs_t* piksi_obs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    piksi_obs->sender_id = mavlink_msg_piksi_obs_get_sender_id(msg);
    piksi_obs->msg_type = mavlink_msg_piksi_obs_get_msg_type(msg);
    piksi_obs->crc = mavlink_msg_piksi_obs_get_crc(msg);
    piksi_obs->seq = mavlink_msg_piksi_obs_get_seq(msg);
    piksi_obs->len = mavlink_msg_piksi_obs_get_len(msg);
    mavlink_msg_piksi_obs_get_data(msg, piksi_obs->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIKSI_OBS_LEN? msg->len : MAVLINK_MSG_ID_PIKSI_OBS_LEN;
        memset(piksi_obs, 0, MAVLINK_MSG_ID_PIKSI_OBS_LEN);
    memcpy(piksi_obs, _MAV_PAYLOAD(msg), len);
#endif
}
