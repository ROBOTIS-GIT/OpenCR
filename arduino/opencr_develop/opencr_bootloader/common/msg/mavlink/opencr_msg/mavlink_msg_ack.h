// MESSAGE ACK PACKING

#define MAVLINK_MSG_ID_ACK 150

typedef struct MAVLINK_PACKED __mavlink_ack_t
{
 uint16_t err_code; /*< */
 uint8_t msg_id; /*< */
 uint8_t length; /*< */
 uint8_t data[16]; /*< */
} mavlink_ack_t;

#define MAVLINK_MSG_ID_ACK_LEN 20
#define MAVLINK_MSG_ID_ACK_MIN_LEN 20
#define MAVLINK_MSG_ID_150_LEN 20
#define MAVLINK_MSG_ID_150_MIN_LEN 20

#define MAVLINK_MSG_ID_ACK_CRC 192
#define MAVLINK_MSG_ID_150_CRC 192

#define MAVLINK_MSG_ACK_FIELD_DATA_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACK { \
	150, \
	"ACK", \
	4, \
	{  { "err_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_ack_t, err_code) }, \
         { "msg_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_ack_t, msg_id) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_ack_t, length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 16, 4, offsetof(mavlink_ack_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACK { \
	"ACK", \
	4, \
	{  { "err_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_ack_t, err_code) }, \
         { "msg_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_ack_t, msg_id) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_ack_t, length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 16, 4, offsetof(mavlink_ack_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param msg_id 
 * @param err_code 
 * @param length 
 * @param data 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t msg_id, uint16_t err_code, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ACK_LEN];
	_mav_put_uint16_t(buf, 0, err_code);
	_mav_put_uint8_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACK_LEN);
#else
	mavlink_ack_t packet;
	packet.err_code = err_code;
	packet.msg_id = msg_id;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
}

/**
 * @brief Pack a ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param msg_id 
 * @param err_code 
 * @param length 
 * @param data 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t msg_id,uint16_t err_code,uint8_t length,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ACK_LEN];
	_mav_put_uint16_t(buf, 0, err_code);
	_mav_put_uint8_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACK_LEN);
#else
	mavlink_ack_t packet;
	packet.err_code = err_code;
	packet.msg_id = msg_id;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
}

/**
 * @brief Encode a ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ack_t* ack)
{
	return mavlink_msg_ack_pack(system_id, component_id, msg, ack->msg_id, ack->err_code, ack->length, ack->data);
}

/**
 * @brief Encode a ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ack_t* ack)
{
	return mavlink_msg_ack_pack_chan(system_id, component_id, chan, msg, ack->msg_id, ack->err_code, ack->length, ack->data);
}

/**
 * @brief Send a ack message
 * @param chan MAVLink channel to send the message
 *
 * @param msg_id 
 * @param err_code 
 * @param length 
 * @param data 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ack_send(mavlink_channel_t chan, uint8_t msg_id, uint16_t err_code, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ACK_LEN];
	_mav_put_uint16_t(buf, 0, err_code);
	_mav_put_uint8_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, buf, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#else
	mavlink_ack_t packet;
	packet.err_code = err_code;
	packet.msg_id = msg_id;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, (const char *)&packet, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#endif
}

/**
 * @brief Send a ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ack_send_struct(mavlink_channel_t chan, const mavlink_ack_t* ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ack_send(chan, ack->msg_id, ack->err_code, ack->length, ack->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, (const char *)ack, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t msg_id, uint16_t err_code, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, err_code);
	_mav_put_uint8_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, buf, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#else
	mavlink_ack_t *packet = (mavlink_ack_t *)msgbuf;
	packet->err_code = err_code;
	packet->msg_id = msg_id;
	packet->length = length;
	mav_array_memcpy(packet->data, data, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, (const char *)packet, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE ACK UNPACKING


/**
 * @brief Get field msg_id from ack message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_ack_get_msg_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field err_code from ack message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ack_get_err_code(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field length from ack message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_ack_get_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field data from ack message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ack_get_data(const mavlink_message_t* msg, uint8_t *data)
{
	return _MAV_RETURN_uint8_t_array(msg, data, 16,  4);
}

/**
 * @brief Decode a ack message into a struct
 *
 * @param msg The message to decode
 * @param ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_ack_decode(const mavlink_message_t* msg, mavlink_ack_t* ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	ack->err_code = mavlink_msg_ack_get_err_code(msg);
	ack->msg_id = mavlink_msg_ack_get_msg_id(msg);
	ack->length = mavlink_msg_ack_get_length(msg);
	mavlink_msg_ack_get_data(msg, ack->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACK_LEN? msg->len : MAVLINK_MSG_ID_ACK_LEN;
        memset(ack, 0, MAVLINK_MSG_ID_ACK_LEN);
	memcpy(ack, _MAV_PAYLOAD(msg), len);
#endif
}
