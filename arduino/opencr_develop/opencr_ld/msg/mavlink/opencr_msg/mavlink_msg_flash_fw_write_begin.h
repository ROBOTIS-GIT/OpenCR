// MESSAGE FLASH_FW_WRITE_BEGIN PACKING

#define MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN 154

typedef struct MAVLINK_PACKED __mavlink_flash_fw_write_begin_t
{
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t param[8]; /*< */
} mavlink_flash_fw_write_begin_t;

#define MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN 9
#define MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN 9
#define MAVLINK_MSG_ID_154_LEN 9
#define MAVLINK_MSG_ID_154_MIN_LEN 9

#define MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC 8
#define MAVLINK_MSG_ID_154_CRC 8

#define MAVLINK_MSG_FLASH_FW_WRITE_BEGIN_FIELD_PARAM_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLASH_FW_WRITE_BEGIN { \
	154, \
	"FLASH_FW_WRITE_BEGIN", \
	2, \
	{  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_flash_fw_write_begin_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 1, offsetof(mavlink_flash_fw_write_begin_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLASH_FW_WRITE_BEGIN { \
	"FLASH_FW_WRITE_BEGIN", \
	2, \
	{  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_flash_fw_write_begin_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 1, offsetof(mavlink_flash_fw_write_begin_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a flash_fw_write_begin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_write_begin_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t resp, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN];
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t_array(buf, 1, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN);
#else
	mavlink_flash_fw_write_begin_t packet;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC);
}

/**
 * @brief Pack a flash_fw_write_begin message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_write_begin_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t resp,const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN];
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t_array(buf, 1, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN);
#else
	mavlink_flash_fw_write_begin_t packet;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC);
}

/**
 * @brief Encode a flash_fw_write_begin struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_write_begin C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_write_begin_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flash_fw_write_begin_t* flash_fw_write_begin)
{
	return mavlink_msg_flash_fw_write_begin_pack(system_id, component_id, msg, flash_fw_write_begin->resp, flash_fw_write_begin->param);
}

/**
 * @brief Encode a flash_fw_write_begin struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_write_begin C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_write_begin_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flash_fw_write_begin_t* flash_fw_write_begin)
{
	return mavlink_msg_flash_fw_write_begin_pack_chan(system_id, component_id, chan, msg, flash_fw_write_begin->resp, flash_fw_write_begin->param);
}

/**
 * @brief Send a flash_fw_write_begin message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param param 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flash_fw_write_begin_send(mavlink_channel_t chan, uint8_t resp, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN];
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t_array(buf, 1, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN, buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC);
#else
	mavlink_flash_fw_write_begin_t packet;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN, (const char *)&packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC);
#endif
}

/**
 * @brief Send a flash_fw_write_begin message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flash_fw_write_begin_send_struct(mavlink_channel_t chan, const mavlink_flash_fw_write_begin_t* flash_fw_write_begin)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flash_fw_write_begin_send(chan, flash_fw_write_begin->resp, flash_fw_write_begin->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN, (const char *)flash_fw_write_begin, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flash_fw_write_begin_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, resp);
	_mav_put_uint8_t_array(buf, 1, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN, buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC);
#else
	mavlink_flash_fw_write_begin_t *packet = (mavlink_flash_fw_write_begin_t *)msgbuf;
	packet->resp = resp;
	mav_array_memcpy(packet->param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN, (const char *)packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_CRC);
#endif
}
#endif

#endif

// MESSAGE FLASH_FW_WRITE_BEGIN UNPACKING


/**
 * @brief Get field resp from flash_fw_write_begin message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_flash_fw_write_begin_get_resp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param from flash_fw_write_begin message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_write_begin_get_param(const mavlink_message_t* msg, uint8_t *param)
{
	return _MAV_RETURN_uint8_t_array(msg, param, 8,  1);
}

/**
 * @brief Decode a flash_fw_write_begin message into a struct
 *
 * @param msg The message to decode
 * @param flash_fw_write_begin C-struct to decode the message contents into
 */
static inline void mavlink_msg_flash_fw_write_begin_decode(const mavlink_message_t* msg, mavlink_flash_fw_write_begin_t* flash_fw_write_begin)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	flash_fw_write_begin->resp = mavlink_msg_flash_fw_write_begin_get_resp(msg);
	mavlink_msg_flash_fw_write_begin_get_param(msg, flash_fw_write_begin->param);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN? msg->len : MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN;
        memset(flash_fw_write_begin, 0, MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN_LEN);
	memcpy(flash_fw_write_begin, _MAV_PAYLOAD(msg), len);
#endif
}
