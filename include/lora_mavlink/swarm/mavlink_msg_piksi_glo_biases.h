#pragma once
// MESSAGE PIKSI_GLO_BIASES PACKING

#define MAVLINK_MSG_ID_PIKSI_GLO_BIASES 186


typedef struct __mavlink_piksi_glo_biases_t {
 uint16_t sender_id; /*<  glo biases sender id*/
 uint16_t msg_type; /*<  msg_type*/
 uint16_t crc; /*<  sbp crc*/
 uint8_t seq; /*<  message sequence*/
 uint8_t len; /*<  glo biases data length*/
 uint8_t data[9]; /*<  glo biases data*/
} mavlink_piksi_glo_biases_t;

#define MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN 17
#define MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN 17
#define MAVLINK_MSG_ID_186_LEN 17
#define MAVLINK_MSG_ID_186_MIN_LEN 17

#define MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC 188
#define MAVLINK_MSG_ID_186_CRC 188

#define MAVLINK_MSG_PIKSI_GLO_BIASES_FIELD_DATA_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIKSI_GLO_BIASES { \
    186, \
    "PIKSI_GLO_BIASES", \
    6, \
    {  { "sender_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_piksi_glo_biases_t, sender_id) }, \
         { "msg_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_piksi_glo_biases_t, msg_type) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_piksi_glo_biases_t, seq) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_piksi_glo_biases_t, crc) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_piksi_glo_biases_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 9, 8, offsetof(mavlink_piksi_glo_biases_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIKSI_GLO_BIASES { \
    "PIKSI_GLO_BIASES", \
    6, \
    {  { "sender_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_piksi_glo_biases_t, sender_id) }, \
         { "msg_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_piksi_glo_biases_t, msg_type) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_piksi_glo_biases_t, seq) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_piksi_glo_biases_t, crc) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_piksi_glo_biases_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 9, 8, offsetof(mavlink_piksi_glo_biases_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a piksi_glo_biases message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sender_id  glo biases sender id
 * @param msg_type  msg_type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  glo biases data length
 * @param data  glo biases data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN);
#else
    mavlink_piksi_glo_biases_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_GLO_BIASES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC);
}

/**
 * @brief Pack a piksi_glo_biases message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sender_id  glo biases sender id
 * @param msg_type  msg_type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  glo biases data length
 * @param data  glo biases data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t sender_id,uint16_t msg_type,uint8_t seq,uint16_t crc,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN);
#else
    mavlink_piksi_glo_biases_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_GLO_BIASES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC);
}

/**
 * @brief Encode a piksi_glo_biases struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param piksi_glo_biases C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_piksi_glo_biases_t* piksi_glo_biases)
{
    return mavlink_msg_piksi_glo_biases_pack(system_id, component_id, msg, piksi_glo_biases->sender_id, piksi_glo_biases->msg_type, piksi_glo_biases->seq, piksi_glo_biases->crc, piksi_glo_biases->len, piksi_glo_biases->data);
}

/**
 * @brief Encode a piksi_glo_biases struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param piksi_glo_biases C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_piksi_glo_biases_t* piksi_glo_biases)
{
    return mavlink_msg_piksi_glo_biases_pack_chan(system_id, component_id, chan, msg, piksi_glo_biases->sender_id, piksi_glo_biases->msg_type, piksi_glo_biases->seq, piksi_glo_biases->crc, piksi_glo_biases->len, piksi_glo_biases->data);
}

/**
 * @brief Send a piksi_glo_biases message
 * @param chan MAVLink channel to send the message
 *
 * @param sender_id  glo biases sender id
 * @param msg_type  msg_type
 * @param seq  message sequence
 * @param crc  sbp crc
 * @param len  glo biases data length
 * @param data  glo biases data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_piksi_glo_biases_send(mavlink_channel_t chan, uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN];
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_GLO_BIASES, buf, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC);
#else
    mavlink_piksi_glo_biases_t packet;
    packet.sender_id = sender_id;
    packet.msg_type = msg_type;
    packet.crc = crc;
    packet.seq = seq;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_GLO_BIASES, (const char *)&packet, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC);
#endif
}

/**
 * @brief Send a piksi_glo_biases message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_piksi_glo_biases_send_struct(mavlink_channel_t chan, const mavlink_piksi_glo_biases_t* piksi_glo_biases)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_piksi_glo_biases_send(chan, piksi_glo_biases->sender_id, piksi_glo_biases->msg_type, piksi_glo_biases->seq, piksi_glo_biases->crc, piksi_glo_biases->len, piksi_glo_biases->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_GLO_BIASES, (const char *)piksi_glo_biases, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_piksi_glo_biases_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t sender_id, uint16_t msg_type, uint8_t seq, uint16_t crc, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, sender_id);
    _mav_put_uint16_t(buf, 2, msg_type);
    _mav_put_uint16_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_GLO_BIASES, buf, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC);
#else
    mavlink_piksi_glo_biases_t *packet = (mavlink_piksi_glo_biases_t *)msgbuf;
    packet->sender_id = sender_id;
    packet->msg_type = msg_type;
    packet->crc = crc;
    packet->seq = seq;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_GLO_BIASES, (const char *)packet, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_MIN_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_CRC);
#endif
}
#endif

#endif

// MESSAGE PIKSI_GLO_BIASES UNPACKING


/**
 * @brief Get field sender_id from piksi_glo_biases message
 *
 * @return  glo biases sender id
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_get_sender_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field msg_type from piksi_glo_biases message
 *
 * @return  msg_type
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_get_msg_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field seq from piksi_glo_biases message
 *
 * @return  message sequence
 */
static inline uint8_t mavlink_msg_piksi_glo_biases_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field crc from piksi_glo_biases message
 *
 * @return  sbp crc
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_get_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field len from piksi_glo_biases message
 *
 * @return  glo biases data length
 */
static inline uint8_t mavlink_msg_piksi_glo_biases_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field data from piksi_glo_biases message
 *
 * @return  glo biases data
 */
static inline uint16_t mavlink_msg_piksi_glo_biases_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 9,  8);
}

/**
 * @brief Decode a piksi_glo_biases message into a struct
 *
 * @param msg The message to decode
 * @param piksi_glo_biases C-struct to decode the message contents into
 */
static inline void mavlink_msg_piksi_glo_biases_decode(const mavlink_message_t* msg, mavlink_piksi_glo_biases_t* piksi_glo_biases)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    piksi_glo_biases->sender_id = mavlink_msg_piksi_glo_biases_get_sender_id(msg);
    piksi_glo_biases->msg_type = mavlink_msg_piksi_glo_biases_get_msg_type(msg);
    piksi_glo_biases->crc = mavlink_msg_piksi_glo_biases_get_crc(msg);
    piksi_glo_biases->seq = mavlink_msg_piksi_glo_biases_get_seq(msg);
    piksi_glo_biases->len = mavlink_msg_piksi_glo_biases_get_len(msg);
    mavlink_msg_piksi_glo_biases_get_data(msg, piksi_glo_biases->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN? msg->len : MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN;
        memset(piksi_glo_biases, 0, MAVLINK_MSG_ID_PIKSI_GLO_BIASES_LEN);
    memcpy(piksi_glo_biases, _MAV_PAYLOAD(msg), len);
#endif
}
