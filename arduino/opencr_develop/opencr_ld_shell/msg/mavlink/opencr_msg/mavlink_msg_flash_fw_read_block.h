// MESSAGE FLASH_FW_READ_BLOCK PACKING

#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK 161

typedef struct MAVLINK_PACKED __mavlink_flash_fw_read_block_t
{
 uint32_t addr; /*< */
 uint16_t length; /*< */
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
} mavlink_flash_fw_read_block_t;

#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN 7
#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN 7
#define MAVLINK_MSG_ID_161_LEN 7
#define MAVLINK_MSG_ID_161_MIN_LEN 7

#define MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC 131
#define MAVLINK_MSG_ID_161_CRC 131



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLASH_FW_READ_BLOCK { \
	161, \
	"FLASH_FW_READ_BLOCK", \
	3, \
	{  { "addr", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_flash_fw_read_block_t, addr) }, \
         { "length", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_flash_fw_read_block_t, length) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_flash_fw_read_block_t, resp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLASH_FW_READ_BLOCK { \
	"FLASH_FW_READ_BLOCK", \
	3, \
	{  { "addr", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_flash_fw_read_block_t, addr) }, \
         { "length", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_flash_fw_read_block_t, length) }, \
         { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_flash_fw_read_block_t, resp) }, \
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_read_block_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t resp, uint32_t addr, uint16_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN];
	_mav_put_uint32_t(buf, 0, addr);
	_mav_put_uint16_t(buf, 4, length);
	_mav_put_uint8_t(buf, 6, resp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
#else
	mavlink_flash_fw_read_block_t packet;
	packet.addr = addr;
	packet.length = length;
	packet.resp = resp;

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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flash_fw_read_block_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t resp,uint32_t addr,uint16_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN];
	_mav_put_uint32_t(buf, 0, addr);
	_mav_put_uint16_t(buf, 4, length);
	_mav_put_uint8_t(buf, 6, resp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
#else
	mavlink_flash_fw_read_block_t packet;
	packet.addr = addr;
	packet.length = length;
	packet.resp = resp;

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
	return mavlink_msg_flash_fw_read_block_pack(system_id, component_id, msg, flash_fw_read_block->resp, flash_fw_read_block->addr, flash_fw_read_block->length);
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
	return mavlink_msg_flash_fw_read_block_pack_chan(system_id, component_id, chan, msg, flash_fw_read_block->resp, flash_fw_read_block->addr, flash_fw_read_block->length);
}

/**
 * @brief Send a flash_fw_read_block message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param addr 
 * @param length 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flash_fw_read_block_send(mavlink_channel_t chan, uint8_t resp, uint32_t addr, uint16_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN];
	_mav_put_uint32_t(buf, 0, addr);
	_mav_put_uint16_t(buf, 4, length);
	_mav_put_uint8_t(buf, 6, resp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK, buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
#else
	mavlink_flash_fw_read_block_t packet;
	packet.addr = addr;
	packet.length = length;
	packet.resp = resp;

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
    mavlink_msg_flash_fw_read_block_send(chan, flash_fw_read_block->resp, flash_fw_read_block->addr, flash_fw_read_block->length);
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
static inline void mavlink_msg_flash_fw_read_block_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint32_t addr, uint16_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, addr);
	_mav_put_uint16_t(buf, 4, length);
	_mav_put_uint8_t(buf, 6, resp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK, buf, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_MIN_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_CRC);
#else
	mavlink_flash_fw_read_block_t *packet = (mavlink_flash_fw_read_block_t *)msgbuf;
	packet->addr = addr;
	packet->length = length;
	packet->resp = resp;

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
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field addr from flash_fw_read_block message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_flash_fw_read_block_get_addr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field length from flash_fw_read_block message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_flash_fw_read_block_get_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
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
	flash_fw_read_block->length = mavlink_msg_flash_fw_read_block_get_length(msg);
	flash_fw_read_block->resp = mavlink_msg_flash_fw_read_block_get_resp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN? msg->len : MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN;
        memset(flash_fw_read_block, 0, MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK_LEN);
	memcpy(flash_fw_read_block, _MAV_PAYLOAD(msg), len);
#endif
}
