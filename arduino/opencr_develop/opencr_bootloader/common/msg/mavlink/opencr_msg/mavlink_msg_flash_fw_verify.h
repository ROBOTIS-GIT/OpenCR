// MESSAGE FLASH_FW_VERIFY PACKING

#define MAVLINK_MSG_ID_FLASH_FW_VERIFY 159

typedef struct MAVLINK_PACKED __mavlink_flash_fw_verify_t
{
 uint32_t length; /*< */
 uint32_t crc; /*< */
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t param[8]; /*< */
} mavlink_flash_fw_verify_t;

#define MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN 17
#define MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN 17
#define MAVLINK_MSG_ID_159_LEN 17
#define MAVLINK_MSG_ID_159_MIN_LEN 17

#define MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC 31
#define MAVLINK_MSG_ID_159_CRC 31

#define MAVLINK_MSG_FLASH_FW_VERIFY_FIELD_PARAM_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLASH_FW_VERIFY { \
	159, \
	"FLASH_FW_VERIFY", \
	4, \
	{  { "length", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_flash_fw_verify_t, length) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_flash_fw_verify_t, crc) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_flash_fw_verify_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 9, offsetof(mavlink_flash_fw_verify_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLASH_FW_VERIFY { \
	"FLASH_FW_VERIFY", \
	4, \
	{  { "length", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_flash_fw_verify_t, length) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_flash_fw_verify_t, crc) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_flash_fw_verify_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 9, offsetof(mavlink_flash_fw_verify_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a flash_fw_verify message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param length 
 * @param crc 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_verify_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t resp, uint32_t length, uint32_t crc, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN];
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint32_t(buf, 4, crc);
	_mav_put_uint8_t(buf, 8, resp);
	_mav_put_uint8_t_array(buf, 9, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN);
#else
	mavlink_flash_fw_verify_t packet;
	packet.length = length;
	packet.crc = crc;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_VERIFY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC);
}

/**
 * @brief Pack a flash_fw_verify message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param length 
 * @param crc 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_verify_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t resp,uint32_t length,uint32_t crc,const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN];
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint32_t(buf, 4, crc);
	_mav_put_uint8_t(buf, 8, resp);
	_mav_put_uint8_t_array(buf, 9, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN);
#else
	mavlink_flash_fw_verify_t packet;
	packet.length = length;
	packet.crc = crc;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_VERIFY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC);
}

/**
 * @brief Encode a flash_fw_verify struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_verify C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_verify_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flash_fw_verify_t* flash_fw_verify)
{
	return mavlink_msg_flash_fw_verify_pack(system_id, component_id, msg, flash_fw_verify->resp, flash_fw_verify->length, flash_fw_verify->crc, flash_fw_verify->param);
}

/**
 * @brief Encode a flash_fw_verify struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_verify C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_verify_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flash_fw_verify_t* flash_fw_verify)
{
	return mavlink_msg_flash_fw_verify_pack_chan(system_id, component_id, chan, msg, flash_fw_verify->resp, flash_fw_verify->length, flash_fw_verify->crc, flash_fw_verify->param);
}

/**
 * @brief Send a flash_fw_verify message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param length 
 * @param crc 
 * @param param 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flash_fw_verify_send(mavlink_channel_t chan, uint8_t resp, uint32_t length, uint32_t crc, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN];
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint32_t(buf, 4, crc);
	_mav_put_uint8_t(buf, 8, resp);
	_mav_put_uint8_t_array(buf, 9, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_VERIFY, buf, MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC);
#else
	mavlink_flash_fw_verify_t packet;
	packet.length = length;
	packet.crc = crc;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_VERIFY, (const char *)&packet, MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC);
#endif
}

/**
 * @brief Send a flash_fw_verify message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flash_fw_verify_send_struct(mavlink_channel_t chan, const mavlink_flash_fw_verify_t* flash_fw_verify)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flash_fw_verify_send(chan, flash_fw_verify->resp, flash_fw_verify->length, flash_fw_verify->crc, flash_fw_verify->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_VERIFY, (const char *)flash_fw_verify, MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flash_fw_verify_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint32_t length, uint32_t crc, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint32_t(buf, 4, crc);
	_mav_put_uint8_t(buf, 8, resp);
	_mav_put_uint8_t_array(buf, 9, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_VERIFY, buf, MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC);
#else
	mavlink_flash_fw_verify_t *packet = (mavlink_flash_fw_verify_t *)msgbuf;
	packet->length = length;
	packet->crc = crc;
	packet->resp = resp;
	mav_array_memcpy(packet->param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_VERIFY, (const char *)packet, MAVLINK_MSG_ID_FLASH_FW_VERIFY_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN, MAVLINK_MSG_ID_FLASH_FW_VERIFY_CRC);
#endif
}
#endif

#endif

// MESSAGE FLASH_FW_VERIFY UNPACKING


/**
 * @brief Get field resp from flash_fw_verify message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_flash_fw_verify_get_resp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field length from flash_fw_verify message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_flash_fw_verify_get_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field crc from flash_fw_verify message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_flash_fw_verify_get_crc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field param from flash_fw_verify message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_verify_get_param(const mavlink_message_t* msg, uint8_t *param)
{
	return _MAV_RETURN_uint8_t_array(msg, param, 8,  9);
}

/**
 * @brief Decode a flash_fw_verify message into a struct
 *
 * @param msg The message to decode
 * @param flash_fw_verify C-struct to decode the message contents into
 */
static inline void mavlink_msg_flash_fw_verify_decode(const mavlink_message_t* msg, mavlink_flash_fw_verify_t* flash_fw_verify)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	flash_fw_verify->length = mavlink_msg_flash_fw_verify_get_length(msg);
	flash_fw_verify->crc = mavlink_msg_flash_fw_verify_get_crc(msg);
	flash_fw_verify->resp = mavlink_msg_flash_fw_verify_get_resp(msg);
	mavlink_msg_flash_fw_verify_get_param(msg, flash_fw_verify->param);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN? msg->len : MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN;
        memset(flash_fw_verify, 0, MAVLINK_MSG_ID_FLASH_FW_VERIFY_LEN);
	memcpy(flash_fw_verify, _MAV_PAYLOAD(msg), len);
#endif
}
