// MESSAGE READ_TAG PACKING

#define MAVLINK_MSG_ID_READ_TAG 153

typedef struct MAVLINK_PACKED __mavlink_read_tag_t
{
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t type; /*< */
 uint8_t param[8]; /*< */
} mavlink_read_tag_t;

#define MAVLINK_MSG_ID_READ_TAG_LEN 10
#define MAVLINK_MSG_ID_READ_TAG_MIN_LEN 10
#define MAVLINK_MSG_ID_153_LEN 10
#define MAVLINK_MSG_ID_153_MIN_LEN 10

#define MAVLINK_MSG_ID_READ_TAG_CRC 126
#define MAVLINK_MSG_ID_153_CRC 126

#define MAVLINK_MSG_READ_TAG_FIELD_PARAM_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_READ_TAG { \
	153, \
	"READ_TAG", \
	3, \
	{  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_read_tag_t, resp) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_read_tag_t, type) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 2, offsetof(mavlink_read_tag_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_READ_TAG { \
	"READ_TAG", \
	3, \
	{  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_read_tag_t, resp) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_read_tag_t, type) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 2, offsetof(mavlink_read_tag_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a read_tag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param type 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_read_tag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t resp, uint8_t type, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_READ_TAG_LEN];
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t(buf, 1, type);
	_mav_put_uint8_t_array(buf, 2, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_READ_TAG_LEN);
#else
	mavlink_read_tag_t packet;
	packet.resp = resp;
	packet.type = type;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_READ_TAG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_READ_TAG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_READ_TAG_MIN_LEN, MAVLINK_MSG_ID_READ_TAG_LEN, MAVLINK_MSG_ID_READ_TAG_CRC);
}

/**
 * @brief Pack a read_tag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param type 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_read_tag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t resp,uint8_t type,const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_READ_TAG_LEN];
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t(buf, 1, type);
	_mav_put_uint8_t_array(buf, 2, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_READ_TAG_LEN);
#else
	mavlink_read_tag_t packet;
	packet.resp = resp;
	packet.type = type;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_READ_TAG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_READ_TAG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_READ_TAG_MIN_LEN, MAVLINK_MSG_ID_READ_TAG_LEN, MAVLINK_MSG_ID_READ_TAG_CRC);
}

/**
 * @brief Encode a read_tag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param read_tag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_read_tag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_read_tag_t* read_tag)
{
	return mavlink_msg_read_tag_pack(system_id, component_id, msg, read_tag->resp, read_tag->type, read_tag->param);
}

/**
 * @brief Encode a read_tag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param read_tag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_read_tag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_read_tag_t* read_tag)
{
	return mavlink_msg_read_tag_pack_chan(system_id, component_id, chan, msg, read_tag->resp, read_tag->type, read_tag->param);
}

/**
 * @brief Send a read_tag message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param type 
 * @param param 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_read_tag_send(mavlink_channel_t chan, uint8_t resp, uint8_t type, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_READ_TAG_LEN];
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t(buf, 1, type);
	_mav_put_uint8_t_array(buf, 2, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_READ_TAG, buf, MAVLINK_MSG_ID_READ_TAG_MIN_LEN, MAVLINK_MSG_ID_READ_TAG_LEN, MAVLINK_MSG_ID_READ_TAG_CRC);
#else
	mavlink_read_tag_t packet;
	packet.resp = resp;
	packet.type = type;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_READ_TAG, (const char *)&packet, MAVLINK_MSG_ID_READ_TAG_MIN_LEN, MAVLINK_MSG_ID_READ_TAG_LEN, MAVLINK_MSG_ID_READ_TAG_CRC);
#endif
}

/**
 * @brief Send a read_tag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_read_tag_send_struct(mavlink_channel_t chan, const mavlink_read_tag_t* read_tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_read_tag_send(chan, read_tag->resp, read_tag->type, read_tag->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_READ_TAG, (const char *)read_tag, MAVLINK_MSG_ID_READ_TAG_MIN_LEN, MAVLINK_MSG_ID_READ_TAG_LEN, MAVLINK_MSG_ID_READ_TAG_CRC);
#endif
}

#if MAVLINK_MSG_ID_READ_TAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_read_tag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint8_t type, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t(buf, 1, type);
	_mav_put_uint8_t_array(buf, 2, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_READ_TAG, buf, MAVLINK_MSG_ID_READ_TAG_MIN_LEN, MAVLINK_MSG_ID_READ_TAG_LEN, MAVLINK_MSG_ID_READ_TAG_CRC);
#else
	mavlink_read_tag_t *packet = (mavlink_read_tag_t *)msgbuf;
	packet->resp = resp;
	packet->type = type;
	mav_array_memcpy(packet->param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_READ_TAG, (const char *)packet, MAVLINK_MSG_ID_READ_TAG_MIN_LEN, MAVLINK_MSG_ID_READ_TAG_LEN, MAVLINK_MSG_ID_READ_TAG_CRC);
#endif
}
#endif

#endif

// MESSAGE READ_TAG UNPACKING


/**
 * @brief Get field resp from read_tag message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_read_tag_get_resp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field type from read_tag message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_read_tag_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field param from read_tag message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_read_tag_get_param(const mavlink_message_t* msg, uint8_t *param)
{
	return _MAV_RETURN_uint8_t_array(msg, param, 8,  2);
}

/**
 * @brief Decode a read_tag message into a struct
 *
 * @param msg The message to decode
 * @param read_tag C-struct to decode the message contents into
 */
static inline void mavlink_msg_read_tag_decode(const mavlink_message_t* msg, mavlink_read_tag_t* read_tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	read_tag->resp = mavlink_msg_read_tag_get_resp(msg);
	read_tag->type = mavlink_msg_read_tag_get_type(msg);
	mavlink_msg_read_tag_get_param(msg, read_tag->param);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_READ_TAG_LEN? msg->len : MAVLINK_MSG_ID_READ_TAG_LEN;
        memset(read_tag, 0, MAVLINK_MSG_ID_READ_TAG_LEN);
	memcpy(read_tag, _MAV_PAYLOAD(msg), len);
#endif
}
