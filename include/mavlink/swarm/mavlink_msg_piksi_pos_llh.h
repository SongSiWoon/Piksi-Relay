#pragma once
// MESSAGE PIKSI_POS_LLH PACKING

#define MAVLINK_MSG_ID_PIKSI_POS_LLH 187


typedef struct __mavlink_piksi_pos_llh_t {
 double lat; /*<  lat*/
 double lon; /*<  lon*/
 double height; /*<  height*/
 uint16_t h_accuracy; /*<  h_accuracy*/
 uint16_t v_accuracy; /*<  v_accuracy*/
 uint8_t n_sats; /*<  n_sats*/
 uint8_t flags; /*<  flags*/
} mavlink_piksi_pos_llh_t;

#define MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN 30
#define MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN 30
#define MAVLINK_MSG_ID_187_LEN 30
#define MAVLINK_MSG_ID_187_MIN_LEN 30

#define MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC 52
#define MAVLINK_MSG_ID_187_CRC 52



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIKSI_POS_LLH { \
    187, \
    "PIKSI_POS_LLH", \
    7, \
    {  { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_piksi_pos_llh_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_piksi_pos_llh_t, lon) }, \
         { "height", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_piksi_pos_llh_t, height) }, \
         { "h_accuracy", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_piksi_pos_llh_t, h_accuracy) }, \
         { "v_accuracy", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_piksi_pos_llh_t, v_accuracy) }, \
         { "n_sats", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_piksi_pos_llh_t, n_sats) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_piksi_pos_llh_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIKSI_POS_LLH { \
    "PIKSI_POS_LLH", \
    7, \
    {  { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_piksi_pos_llh_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_piksi_pos_llh_t, lon) }, \
         { "height", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_piksi_pos_llh_t, height) }, \
         { "h_accuracy", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_piksi_pos_llh_t, h_accuracy) }, \
         { "v_accuracy", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_piksi_pos_llh_t, v_accuracy) }, \
         { "n_sats", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_piksi_pos_llh_t, n_sats) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_piksi_pos_llh_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a piksi_pos_llh message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  lat
 * @param lon  lon
 * @param height  height
 * @param h_accuracy  h_accuracy
 * @param v_accuracy  v_accuracy
 * @param n_sats  n_sats
 * @param flags  flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_pos_llh_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double lat, double lon, double height, uint16_t h_accuracy, uint16_t v_accuracy, uint8_t n_sats, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, height);
    _mav_put_uint16_t(buf, 24, h_accuracy);
    _mav_put_uint16_t(buf, 26, v_accuracy);
    _mav_put_uint8_t(buf, 28, n_sats);
    _mav_put_uint8_t(buf, 29, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN);
#else
    mavlink_piksi_pos_llh_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.height = height;
    packet.h_accuracy = h_accuracy;
    packet.v_accuracy = v_accuracy;
    packet.n_sats = n_sats;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_POS_LLH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC);
}

/**
 * @brief Pack a piksi_pos_llh message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat  lat
 * @param lon  lon
 * @param height  height
 * @param h_accuracy  h_accuracy
 * @param v_accuracy  v_accuracy
 * @param n_sats  n_sats
 * @param flags  flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_piksi_pos_llh_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double lat,double lon,double height,uint16_t h_accuracy,uint16_t v_accuracy,uint8_t n_sats,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, height);
    _mav_put_uint16_t(buf, 24, h_accuracy);
    _mav_put_uint16_t(buf, 26, v_accuracy);
    _mav_put_uint8_t(buf, 28, n_sats);
    _mav_put_uint8_t(buf, 29, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN);
#else
    mavlink_piksi_pos_llh_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.height = height;
    packet.h_accuracy = h_accuracy;
    packet.v_accuracy = v_accuracy;
    packet.n_sats = n_sats;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PIKSI_POS_LLH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC);
}

/**
 * @brief Encode a piksi_pos_llh struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param piksi_pos_llh C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_pos_llh_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_piksi_pos_llh_t* piksi_pos_llh)
{
    return mavlink_msg_piksi_pos_llh_pack(system_id, component_id, msg, piksi_pos_llh->lat, piksi_pos_llh->lon, piksi_pos_llh->height, piksi_pos_llh->h_accuracy, piksi_pos_llh->v_accuracy, piksi_pos_llh->n_sats, piksi_pos_llh->flags);
}

/**
 * @brief Encode a piksi_pos_llh struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param piksi_pos_llh C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_piksi_pos_llh_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_piksi_pos_llh_t* piksi_pos_llh)
{
    return mavlink_msg_piksi_pos_llh_pack_chan(system_id, component_id, chan, msg, piksi_pos_llh->lat, piksi_pos_llh->lon, piksi_pos_llh->height, piksi_pos_llh->h_accuracy, piksi_pos_llh->v_accuracy, piksi_pos_llh->n_sats, piksi_pos_llh->flags);
}

/**
 * @brief Send a piksi_pos_llh message
 * @param chan MAVLink channel to send the message
 *
 * @param lat  lat
 * @param lon  lon
 * @param height  height
 * @param h_accuracy  h_accuracy
 * @param v_accuracy  v_accuracy
 * @param n_sats  n_sats
 * @param flags  flags
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_piksi_pos_llh_send(mavlink_channel_t chan, double lat, double lon, double height, uint16_t h_accuracy, uint16_t v_accuracy, uint8_t n_sats, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, height);
    _mav_put_uint16_t(buf, 24, h_accuracy);
    _mav_put_uint16_t(buf, 26, v_accuracy);
    _mav_put_uint8_t(buf, 28, n_sats);
    _mav_put_uint8_t(buf, 29, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_POS_LLH, buf, MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC);
#else
    mavlink_piksi_pos_llh_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.height = height;
    packet.h_accuracy = h_accuracy;
    packet.v_accuracy = v_accuracy;
    packet.n_sats = n_sats;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_POS_LLH, (const char *)&packet, MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC);
#endif
}

/**
 * @brief Send a piksi_pos_llh message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_piksi_pos_llh_send_struct(mavlink_channel_t chan, const mavlink_piksi_pos_llh_t* piksi_pos_llh)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_piksi_pos_llh_send(chan, piksi_pos_llh->lat, piksi_pos_llh->lon, piksi_pos_llh->height, piksi_pos_llh->h_accuracy, piksi_pos_llh->v_accuracy, piksi_pos_llh->n_sats, piksi_pos_llh->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_POS_LLH, (const char *)piksi_pos_llh, MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_piksi_pos_llh_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double lat, double lon, double height, uint16_t h_accuracy, uint16_t v_accuracy, uint8_t n_sats, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, height);
    _mav_put_uint16_t(buf, 24, h_accuracy);
    _mav_put_uint16_t(buf, 26, v_accuracy);
    _mav_put_uint8_t(buf, 28, n_sats);
    _mav_put_uint8_t(buf, 29, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_POS_LLH, buf, MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC);
#else
    mavlink_piksi_pos_llh_t *packet = (mavlink_piksi_pos_llh_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->height = height;
    packet->h_accuracy = h_accuracy;
    packet->v_accuracy = v_accuracy;
    packet->n_sats = n_sats;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIKSI_POS_LLH, (const char *)packet, MAVLINK_MSG_ID_PIKSI_POS_LLH_MIN_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN, MAVLINK_MSG_ID_PIKSI_POS_LLH_CRC);
#endif
}
#endif

#endif

// MESSAGE PIKSI_POS_LLH UNPACKING


/**
 * @brief Get field lat from piksi_pos_llh message
 *
 * @return  lat
 */
static inline double mavlink_msg_piksi_pos_llh_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field lon from piksi_pos_llh message
 *
 * @return  lon
 */
static inline double mavlink_msg_piksi_pos_llh_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field height from piksi_pos_llh message
 *
 * @return  height
 */
static inline double mavlink_msg_piksi_pos_llh_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field h_accuracy from piksi_pos_llh message
 *
 * @return  h_accuracy
 */
static inline uint16_t mavlink_msg_piksi_pos_llh_get_h_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field v_accuracy from piksi_pos_llh message
 *
 * @return  v_accuracy
 */
static inline uint16_t mavlink_msg_piksi_pos_llh_get_v_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field n_sats from piksi_pos_llh message
 *
 * @return  n_sats
 */
static inline uint8_t mavlink_msg_piksi_pos_llh_get_n_sats(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field flags from piksi_pos_llh message
 *
 * @return  flags
 */
static inline uint8_t mavlink_msg_piksi_pos_llh_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Decode a piksi_pos_llh message into a struct
 *
 * @param msg The message to decode
 * @param piksi_pos_llh C-struct to decode the message contents into
 */
static inline void mavlink_msg_piksi_pos_llh_decode(const mavlink_message_t* msg, mavlink_piksi_pos_llh_t* piksi_pos_llh)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    piksi_pos_llh->lat = mavlink_msg_piksi_pos_llh_get_lat(msg);
    piksi_pos_llh->lon = mavlink_msg_piksi_pos_llh_get_lon(msg);
    piksi_pos_llh->height = mavlink_msg_piksi_pos_llh_get_height(msg);
    piksi_pos_llh->h_accuracy = mavlink_msg_piksi_pos_llh_get_h_accuracy(msg);
    piksi_pos_llh->v_accuracy = mavlink_msg_piksi_pos_llh_get_v_accuracy(msg);
    piksi_pos_llh->n_sats = mavlink_msg_piksi_pos_llh_get_n_sats(msg);
    piksi_pos_llh->flags = mavlink_msg_piksi_pos_llh_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN? msg->len : MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN;
        memset(piksi_pos_llh, 0, MAVLINK_MSG_ID_PIKSI_POS_LLH_LEN);
    memcpy(piksi_pos_llh, _MAV_PAYLOAD(msg), len);
#endif
}
