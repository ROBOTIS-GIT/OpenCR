// MESSAGE FLASH_FW_READ_BLOCK PACKING

#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK 159

typedef struct MAVLINK_PACKED __mavlink_flash_fw_read_block_t
{
 uint16_t addr; /*< */
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t length; /*< */
 uint8_t data[128]; /*< */
} mavlink_flash_fw_read_block_t;

#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN 132
#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN 132
#define MAVLINK_MSG_ID_159_LEN 132
#define MAVLINK_MSG_ID_159_MIN_LEN 132

#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC 178
#define MAVLINK_MSG_ID_159_CRC 178

#define MAVLINK_MSG_FLASH_FW_READ_BLOCK_FIELD_DATA_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLASH_FW_READ_BLOCK { \
	159, \
	"FLASH_FW_READ_BLOCK", \
	4, \
	{  { "addr", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_flash_fw_read_block_t, addr) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flash_fw_read_block_t, resp) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_flash_fw_read_block_t, length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 4, offsetof(mavlink_flash_fw_read_block_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLASH_FW_READ_BLOCK { \
	"FLASH_FW_READ_BLOCK", \
	4, \
	{  { "addr", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_flash_fw_read_block_t, addr) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flash_fw_read_block_t, resp) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_flash_fw_read_block_t, length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 4, offsetof(mavlink_flash_fw_read_block_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a flash_fw_read_block message
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
static inline uint16_t mavlink_msg_flash_fw_read_block_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t resp, uint16_t addr, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN];
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
#else
	mavlink_flash_fw_read_block_t packet;
	packet.addr = addr;
	packet.resp = resp;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
}

/**
 * @brief Pack a flash_fw_read_block message on a channel
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
static inline uint16_t mavlink_msg_flash_fw_read_block_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t resp,uint16_t addr,uint8_t length,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN];
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
#else
	mavlink_flash_fw_read_block_t packet;
	packet.addr = addr;
	packet.resp = resp;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
}

/**
 * @brief Encode a flash_fw_read_block struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_read_block C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_read_block_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flash_fw_read_block_t* flash_fw_read_block)
{
	return mavlink_msg_flash_fw_read_block_pack(system_id, component_id, msg, flash_fw_read_block->resp, flash_fw_read_block->addr, flash_fw_read_block->length, flash_fw_read_block->data);
}

/**
 * @brief Encode a flash_fw_read_block struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flash_fw_read_block C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flash_fw_read_block_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flash_fw_read_block_t* flash_fw_read_block)
{
	return mavlink_msg_flash_fw_read_block_pack_chan(system_id, component_id, chan, msg, flash_fw_read_block->resp, flash_fw_read_block->addr, flash_fw_read_block->length, flash_fw_read_block->data);
}

/**
 * @brief Send a flash_fw_read_block message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param addr 
 * @param length 
 * @param data 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flash_fw_read_block_send(mavlink_channel_t chan, uint8_t resp, uint16_t addr, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN];
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK, buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
#else
	mavlink_flash_fw_read_block_t packet;
	packet.addr = addr;
	packet.resp = resp;
	packet.length = length;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK, (const char *)&packet, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
#endif
}

/**
 * @brief Send a flash_fw_read_block message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flash_fw_read_block_send_struct(mavlink_channel_t chan, const mavlink_flash_fw_read_block_t* flash_fw_read_block)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flash_fw_read_block_send(chan, flash_fw_read_block->resp, flash_fw_read_block->addr, flash_fw_read_block->length, flash_fw_read_block->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK, (const char *)flash_fw_read_block, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flash_fw_read_block_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint16_t addr, uint8_t length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, addr);
	_mav_put_uint8_t(buf, 2, resp);
	_mav_put_uint8_t(buf, 3, length);
	_mav_put_uint8_t_array(buf, 4, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK, buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
#else
	mavlink_flash_fw_read_block_t *packet = (mavlink_flash_fw_read_block_t *)msgbuf;
	packet->addr = addr;
	packet->resp = resp;
	packet->length = length;
	mav_array_memcpy(packet->data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK, (const char *)packet, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
#endif
}
#endif

#endif

// MESSAGE FLASH_FW_READ_BLOCK UNPACKING


/**
 * @brief Get field resp from flash_fw_read_block message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_flash_fw_read_block_get_resp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field addr from flash_fw_read_block message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_read_block_get_addr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field length from flash_fw_read_block message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_flash_fw_read_block_get_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field data from flash_fw_read_block message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_read_block_get_data(const mavlink_message_t* msg, uint8_t *data)
{
	return _MAV_RETURN_uint8_t_array(msg, data, 128,  4);
}

/**
 * @brief Decode a flash_fw_read_block message into a struct
 *
 * @param msg The message to decode
 * @param flash_fw_read_block C-struct to decode the message contents into
 */
static inline void mavlink_msg_flash_fw_read_block_decode(const mavlink_message_t* msg, mavlink_flash_fw_read_block_t* flash_fw_read_block)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	flash_fw_read_block->addr = mavlink_msg_flash_fw_read_block_get_addr(msg);
	flash_fw_read_block->resp = mavlink_msg_flash_fw_read_block_get_resp(msg);
	flash_fw_read_block->length = mavlink_msg_flash_fw_read_block_get_length(msg);
	mavlink_msg_flash_fw_read_block_get_data(msg, flash_fw_read_block->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN? msg->len : MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN;
        memset(flash_fw_read_block, 0, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
	memcpy(flash_fw_read_block, _MAV_PAYLOAD(msg), len);
#endif
}
