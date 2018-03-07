// MESSAGE FLASH_FW_ERASE PACKING

#define MAVLINK_MSG_ID_FLASH_FW_ERASE 158

typedef struct MAVLINK_PACKED __mavlink_flash_fw_erase_t
{
 uint32_t length; /*< */
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t param[8]; /*< */
} mavlink_flash_fw_erase_t;

#define MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN 13
#define MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN 13
#define MAVLINK_MSG_ID_158_LEN 13
#define MAVLINK_MSG_ID_158_MIN_LEN 13

#define MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC 13
#define MAVLINK_MSG_ID_158_CRC 13

#define MAVLINK_MSG_FLASH_FW_ERASE_FIELD_PARAM_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLASH_FW_ERASE { \
	158, \
	"FLASH_FW_ERASE", \
	3, \
	{  { "length", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_flash_fw_erase_t, length) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_flash_fw_erase_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 5, offsetof(mavlink_flash_fw_erase_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLASH_FW_ERASE { \
	"FLASH_FW_ERASE", \
	3, \
	{  { "length", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_flash_fw_erase_t, length) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_flash_fw_erase_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 8, 5, offsetof(mavlink_flash_fw_erase_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a flash_fw_erase message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param length 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_erase_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t resp, uint32_t length, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN];
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint8_t(buf, 4, resp);
	_mav_put_uint8_t_array(buf, 5, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN);
#else
	mavlink_flash_fw_erase_t packet;
	packet.length = length;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_ERASE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC);
}

/**
 * @brief Pack a flash_fw_erase message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param length 
 * @param param 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_erase_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t resp,uint32_t length,const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN];
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint8_t(buf, 4, resp);
	_mav_put_uint8_t_array(buf, 5, param, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN);
#else
	mavlink_flash_fw_erase_t packet;
	packet.length = length;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_ERASE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC);
}

/**
 * @brief Encode a flash_fw_erase struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_erase C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_erase_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flash_fw_erase_t* flash_fw_erase)
{
	return mavlink_msg_flash_fw_erase_pack(system_id, component_id, msg, flash_fw_erase->resp, flash_fw_erase->length, flash_fw_erase->param);
}

/**
 * @brief Encode a flash_fw_erase struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_erase C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_erase_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flash_fw_erase_t* flash_fw_erase)
{
	return mavlink_msg_flash_fw_erase_pack_chan(system_id, component_id, chan, msg, flash_fw_erase->resp, flash_fw_erase->length, flash_fw_erase->param);
}

/**
 * @brief Send a flash_fw_erase message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param length 
 * @param param 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flash_fw_erase_send(mavlink_channel_t chan, uint8_t resp, uint32_t length, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN];
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint8_t(buf, 4, resp);
	_mav_put_uint8_t_array(buf, 5, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_ERASE, buf, MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC);
#else
	mavlink_flash_fw_erase_t packet;
	packet.length = length;
	packet.resp = resp;
	mav_array_memcpy(packet.param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_ERASE, (const char *)&packet, MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC);
#endif
}

/**
 * @brief Send a flash_fw_erase message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flash_fw_erase_send_struct(mavlink_channel_t chan, const mavlink_flash_fw_erase_t* flash_fw_erase)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flash_fw_erase_send(chan, flash_fw_erase->resp, flash_fw_erase->length, flash_fw_erase->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_ERASE, (const char *)flash_fw_erase, MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flash_fw_erase_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint32_t length, const uint8_t *param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, length);
	_mav_put_uint8_t(buf, 4, resp);
	_mav_put_uint8_t_array(buf, 5, param, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_ERASE, buf, MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC);
#else
	mavlink_flash_fw_erase_t *packet = (mavlink_flash_fw_erase_t *)msgbuf;
	packet->length = length;
	packet->resp = resp;
	mav_array_memcpy(packet->param, param, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_ERASE, (const char *)packet, MAVLINK_MSG_ID_FLASH_FW_ERASE_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN, MAVLINK_MSG_ID_FLASH_FW_ERASE_CRC);
#endif
}
#endif

#endif

// MESSAGE FLASH_FW_ERASE UNPACKING


/**
 * @brief Get field resp from flash_fw_erase message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_flash_fw_erase_get_resp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field length from flash_fw_erase message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_flash_fw_erase_get_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field param from flash_fw_erase message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_erase_get_param(const mavlink_message_t* msg, uint8_t *param)
{
	return _MAV_RETURN_uint8_t_array(msg, param, 8,  5);
}

/**
 * @brief Decode a flash_fw_erase message into a struct
 *
 * @param msg The message to decode
 * @param flash_fw_erase C-struct to decode the message contents into
 */
static inline void mavlink_msg_flash_fw_erase_decode(const mavlink_message_t* msg, mavlink_flash_fw_erase_t* flash_fw_erase)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	flash_fw_erase->length = mavlink_msg_flash_fw_erase_get_length(msg);
	flash_fw_erase->resp = mavlink_msg_flash_fw_erase_get_resp(msg);
	mavlink_msg_flash_fw_erase_get_param(msg, flash_fw_erase->param);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN? msg->len : MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN;
        memset(flash_fw_erase, 0, MAVLINK_MSG_ID_FLASH_FW_ERASE_LEN);
	memcpy(flash_fw_erase, _MAV_PAYLOAD(msg), len);
#endif
}
