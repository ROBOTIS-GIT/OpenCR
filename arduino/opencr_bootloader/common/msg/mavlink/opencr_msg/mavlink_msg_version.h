// MESSAGE VERSION PACKING

#define MAVLINK_MSG_ID_VERSION 150

typedef struct MAVLINK_PACKED __mavlink_version_t
{
 uint8_t type; /*< 0:Cmd, 1:Ack*/
 uint8_t ver_length; /*< */
 uint8_t ver_stirng[16]; /*< */
} mavlink_version_t;

#define MAVLINK_MSG_ID_VERSION_LEN 18
#define MAVLINK_MSG_ID_VERSION_MIN_LEN 18
#define MAVLINK_MSG_ID_150_LEN 18
#define MAVLINK_MSG_ID_150_MIN_LEN 18

#define MAVLINK_MSG_ID_VERSION_CRC 155
#define MAVLINK_MSG_ID_150_CRC 155

#define MAVLINK_MSG_VERSION_FIELD_VER_STIRNG_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VERSION { \
	150, \
	"VERSION", \
	3, \
	{  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_version_t, type) }, \
         { "ver_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_version_t, ver_length) }, \
         { "ver_stirng", NULL, MAVLINK_TYPE_UINT8_T, 16, 2, offsetof(mavlink_version_t, ver_stirng) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VERSION { \
	"VERSION", \
	3, \
	{  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_version_t, type) }, \
         { "ver_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_version_t, ver_length) }, \
         { "ver_stirng", NULL, MAVLINK_TYPE_UINT8_T, 16, 2, offsetof(mavlink_version_t, ver_stirng) }, \
         } \
}
#endif

/**
 * @brief Pack a version message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type 0:Cmd, 1:Ack
 * @param ver_length 
 * @param ver_stirng 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_version_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint8_t ver_length, const uint8_t *ver_stirng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VERSION_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, ver_length);
	_mav_put_uint8_t_array(buf, 2, ver_stirng, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VERSION_LEN);
#else
	mavlink_version_t packet;
	packet.type = type;
	packet.ver_length = ver_length;
	mav_array_memcpy(packet.ver_stirng, ver_stirng, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VERSION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VERSION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VERSION_MIN_LEN, MAVLINK_MSG_ID_VERSION_LEN, MAVLINK_MSG_ID_VERSION_CRC);
}

/**
 * @brief Pack a version message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0:Cmd, 1:Ack
 * @param ver_length 
 * @param ver_stirng 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_version_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,uint8_t ver_length,const uint8_t *ver_stirng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VERSION_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, ver_length);
	_mav_put_uint8_t_array(buf, 2, ver_stirng, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VERSION_LEN);
#else
	mavlink_version_t packet;
	packet.type = type;
	packet.ver_length = ver_length;
	mav_array_memcpy(packet.ver_stirng, ver_stirng, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VERSION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VERSION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VERSION_MIN_LEN, MAVLINK_MSG_ID_VERSION_LEN, MAVLINK_MSG_ID_VERSION_CRC);
}

/**
 * @brief Encode a version struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_version_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_version_t* version)
{
	return mavlink_msg_version_pack(system_id, component_id, msg, version->type, version->ver_length, version->ver_stirng);
}

/**
 * @brief Encode a version struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_version_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_version_t* version)
{
	return mavlink_msg_version_pack_chan(system_id, component_id, chan, msg, version->type, version->ver_length, version->ver_stirng);
}

/**
 * @brief Send a version message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0:Cmd, 1:Ack
 * @param ver_length 
 * @param ver_stirng 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_version_send(mavlink_channel_t chan, uint8_t type, uint8_t ver_length, const uint8_t *ver_stirng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VERSION_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, ver_length);
	_mav_put_uint8_t_array(buf, 2, ver_stirng, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VERSION, buf, MAVLINK_MSG_ID_VERSION_MIN_LEN, MAVLINK_MSG_ID_VERSION_LEN, MAVLINK_MSG_ID_VERSION_CRC);
#else
	mavlink_version_t packet;
	packet.type = type;
	packet.ver_length = ver_length;
	mav_array_memcpy(packet.ver_stirng, ver_stirng, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VERSION, (const char *)&packet, MAVLINK_MSG_ID_VERSION_MIN_LEN, MAVLINK_MSG_ID_VERSION_LEN, MAVLINK_MSG_ID_VERSION_CRC);
#endif
}

/**
 * @brief Send a version message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_version_send_struct(mavlink_channel_t chan, const mavlink_version_t* version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_version_send(chan, version->type, version->ver_length, version->ver_stirng);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VERSION, (const char *)version, MAVLINK_MSG_ID_VERSION_MIN_LEN, MAVLINK_MSG_ID_VERSION_LEN, MAVLINK_MSG_ID_VERSION_CRC);
#endif
}

#if MAVLINK_MSG_ID_VERSION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_version_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint8_t ver_length, const uint8_t *ver_stirng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, ver_length);
	_mav_put_uint8_t_array(buf, 2, ver_stirng, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VERSION, buf, MAVLINK_MSG_ID_VERSION_MIN_LEN, MAVLINK_MSG_ID_VERSION_LEN, MAVLINK_MSG_ID_VERSION_CRC);
#else
	mavlink_version_t *packet = (mavlink_version_t *)msgbuf;
	packet->type = type;
	packet->ver_length = ver_length;
	mav_array_memcpy(packet->ver_stirng, ver_stirng, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VERSION, (const char *)packet, MAVLINK_MSG_ID_VERSION_MIN_LEN, MAVLINK_MSG_ID_VERSION_LEN, MAVLINK_MSG_ID_VERSION_CRC);
#endif
}
#endif

#endif

// MESSAGE VERSION UNPACKING


/**
 * @brief Get field type from version message
 *
 * @return 0:Cmd, 1:Ack
 */
static inline uint8_t mavlink_msg_version_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ver_length from version message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_version_get_ver_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ver_stirng from version message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_version_get_ver_stirng(const mavlink_message_t* msg, uint8_t *ver_stirng)
{
	return _MAV_RETURN_uint8_t_array(msg, ver_stirng, 16,  2);
}

/**
 * @brief Decode a version message into a struct
 *
 * @param msg The message to decode
 * @param version C-struct to decode the message contents into
 */
static inline void mavlink_msg_version_decode(const mavlink_message_t* msg, mavlink_version_t* version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	version->type = mavlink_msg_version_get_type(msg);
	version->ver_length = mavlink_msg_version_get_ver_length(msg);
	mavlink_msg_version_get_ver_stirng(msg, version->ver_stirng);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VERSION_LEN? msg->len : MAVLINK_MSG_ID_VERSION_LEN;
        memset(version, 0, MAVLINK_MSG_ID_VERSION_LEN);
	memcpy(version, _MAV_PAYLOAD(msg), len);
#endif
}
