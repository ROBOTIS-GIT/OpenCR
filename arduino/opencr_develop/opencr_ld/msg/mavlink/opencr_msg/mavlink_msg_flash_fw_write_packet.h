// MESSAGE FLASH_FW_WRITE_PACKET PACKING

#define MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET 156

typedef struct MAVLINK_PACKED __mavlink_flash_fw_write_packet_t
{
 uint16_t addr; /*< */
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t length; /*< */
 uint8_t data[128]; /*< */
} mavlink_flash_fw_write_packet_t;

#define MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN 132
#define MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN 132
#define MAVLINK_MSG_ID_156_LEN 132
#define MAVLINK_MSG_ID_156_MIN_LEN 132

#define MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC 233
#define MAVLINK_MSG_ID_156_CRC 233

#define MAVLINK_MSG_FLASH_FW_WRITE_PACKET_FIELD_DATA_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLASH_FW_WRITE_PACKET { \
	156, \
	"FLASH_FW_WRITE_PACKET", \
	4, \
	{  { "addr", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_flash_fw_write_packet_t, addr) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flash_fw_write_packet_t, resp) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_flash_fw_write_packet_t, length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 4, offsetof(mavlink_flash_fw_write_packet_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLASH_FW_WRITE_PACKET { \
	"FLASH_FW_WRITE_PACKET", \
	4, \
	{  { "addr", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_flash_fw_write_packet_t, addr) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flash_fw_write_packet_t, resp) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_flash_fw_write_packet_t, length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 4, offsetof(mavlink_flash_fw_write_packet_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a flash_fw_write_packet message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param addr 
 * @param length 
 * @param data 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_write_packet_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t resp, uint16_t addr, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN];
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN);
#else
	mavlink_flash_fw_write_packet_t packet;
	packet.addr = addr;
	packet.resp = resp;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC);
}

/**
 * @brief Pack a flash_fw_write_packet message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param addr 
 * @param length 
 * @param data 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_write_packet_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t resp,uint16_t addr,uint8_t length,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN];
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN);
#else
	mavlink_flash_fw_write_packet_t packet;
	packet.addr = addr;
	packet.resp = resp;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC);
}

/**
 * @brief Encode a flash_fw_write_packet struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_write_packet C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_write_packet_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flash_fw_write_packet_t* flash_fw_write_packet)
{
	return mavlink_msg_flash_fw_write_packet_pack(system_id, component_id, msg, flash_fw_write_packet->resp, flash_fw_write_packet->addr, flash_fw_write_packet->length, flash_fw_write_packet->data);
}

/**
 * @brief Encode a flash_fw_write_packet struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_write_packet C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_write_packet_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flash_fw_write_packet_t* flash_fw_write_packet)
{
	return mavlink_msg_flash_fw_write_packet_pack_chan(system_id, component_id, chan, msg, flash_fw_write_packet->resp, flash_fw_write_packet->addr, flash_fw_write_packet->length, flash_fw_write_packet->data);
}

/**
 * @brief Send a flash_fw_write_packet message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param addr 
 * @param length 
 * @param data 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flash_fw_write_packet_send(mavlink_channel_t chan, uint8_t resp, uint16_t addr, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN];
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET, buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC);
#else
	mavlink_flash_fw_write_packet_t packet;
	packet.addr = addr;
	packet.resp = resp;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET, (const char *)&packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC);
#endif
}

/**
 * @brief Send a flash_fw_write_packet message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flash_fw_write_packet_send_struct(mavlink_channel_t chan, const mavlink_flash_fw_write_packet_t* flash_fw_write_packet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flash_fw_write_packet_send(chan, flash_fw_write_packet->resp, flash_fw_write_packet->addr, flash_fw_write_packet->length, flash_fw_write_packet->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET, (const char *)flash_fw_write_packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flash_fw_write_packet_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint16_t addr, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET, buf, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC);
#else
	mavlink_flash_fw_write_packet_t *packet = (mavlink_flash_fw_write_packet_t *)msgbuf;
	packet->addr = addr;
	packet->resp = resp;
	packet->length = length;
	mav_array_memcpy(packet->data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET, (const char *)packet, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_CRC);
#endif
}
#endif

#endif

// MESSAGE FLASH_FW_WRITE_PACKET UNPACKING


/**
 * @brief Get field resp from flash_fw_write_packet message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_flash_fw_write_packet_get_resp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field addr from flash_fw_write_packet message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_write_packet_get_addr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field length from flash_fw_write_packet message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_flash_fw_write_packet_get_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field data from flash_fw_write_packet message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_write_packet_get_data(const mavlink_message_t* msg, uint8_t *data)
{
	return _MAV_RETURN_uint8_t_array(msg, data, 128,  4);
}

/**
 * @brief Decode a flash_fw_write_packet message into a struct
 *
 * @param msg The message to decode
 * @param flash_fw_write_packet C-struct to decode the message contents into
 */
static inline void mavlink_msg_flash_fw_write_packet_decode(const mavlink_message_t* msg, mavlink_flash_fw_write_packet_t* flash_fw_write_packet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	flash_fw_write_packet->addr = mavlink_msg_flash_fw_write_packet_get_addr(msg);
	flash_fw_write_packet->resp = mavlink_msg_flash_fw_write_packet_get_resp(msg);
	flash_fw_write_packet->length = mavlink_msg_flash_fw_write_packet_get_length(msg);
	mavlink_msg_flash_fw_write_packet_get_data(msg, flash_fw_write_packet->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN? msg->len : MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN;
        memset(flash_fw_write_packet, 0, MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET_LEN);
	memcpy(flash_fw_write_packet, _MAV_PAYLOAD(msg), len);
#endif
}
