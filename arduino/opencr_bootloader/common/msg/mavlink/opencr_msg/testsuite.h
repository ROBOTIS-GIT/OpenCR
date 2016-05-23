/** @file
 *	@brief MAVLink comm protocol testsuite generated from opencr_msg.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef OPENCR_MSG_TESTSUITE_H
#define OPENCR_MSG_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_opencr_msg(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_opencr_msg(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACK >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ack_t packet_in = {
		17235,139,{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221 }
    };
	mavlink_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.err_code = packet_in.err_code;
        packet1.msg_id = packet_in.msg_id;
        
        mav_array_memcpy(packet1.param, packet_in.param, sizeof(uint8_t)*16);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ack_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ack_pack(system_id, component_id, &msg , packet1.msg_id , packet1.err_code , packet1.param );
	mavlink_msg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.msg_id , packet1.err_code , packet1.param );
	mavlink_msg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ack_send(MAVLINK_COMM_1 , packet1.msg_id , packet1.err_code , packet1.param );
	mavlink_msg_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_read_version(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_READ_VERSION >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_read_version_t packet_in = {
		5,{ 72, 73, 74, 75, 76, 77, 78, 79 }
    };
	mavlink_read_version_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp = packet_in.resp;
        
        mav_array_memcpy(packet1.param, packet_in.param, sizeof(uint8_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_version_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_read_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_version_pack(system_id, component_id, &msg , packet1.resp , packet1.param );
	mavlink_msg_read_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_version_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp , packet1.param );
	mavlink_msg_read_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_read_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_version_send(MAVLINK_COMM_1 , packet1.resp , packet1.param );
	mavlink_msg_read_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_flash_fw_write_begin(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_flash_fw_write_begin_t packet_in = {
		5,{ 72, 73, 74, 75, 76, 77, 78, 79 }
    };
	mavlink_flash_fw_write_begin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp = packet_in.resp;
        
        mav_array_memcpy(packet1.param, packet_in.param, sizeof(uint8_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_begin_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_flash_fw_write_begin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_begin_pack(system_id, component_id, &msg , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_write_begin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_begin_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_write_begin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_flash_fw_write_begin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_begin_send(MAVLINK_COMM_1 , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_write_begin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_flash_fw_write_end(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FLASH_FW_WRITE_END >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_flash_fw_write_end_t packet_in = {
		5,{ 72, 73, 74, 75, 76, 77, 78, 79 }
    };
	mavlink_flash_fw_write_end_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp = packet_in.resp;
        
        mav_array_memcpy(packet1.param, packet_in.param, sizeof(uint8_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_end_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_flash_fw_write_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_end_pack(system_id, component_id, &msg , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_write_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_end_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_write_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_flash_fw_write_end_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_end_send(MAVLINK_COMM_1 , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_write_end_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_flash_fw_send_block(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FLASH_FW_SEND_BLOCK >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_flash_fw_send_block_t packet_in = {
		17235,139,206,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }
    };
	mavlink_flash_fw_send_block_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.addr = packet_in.addr;
        packet1.resp = packet_in.resp;
        packet1.type = packet_in.type;
        packet1.length = packet_in.length;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*128);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_send_block_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_flash_fw_send_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_send_block_pack(system_id, component_id, &msg , packet1.resp , packet1.type , packet1.addr , packet1.length , packet1.data );
	mavlink_msg_flash_fw_send_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_send_block_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp , packet1.type , packet1.addr , packet1.length , packet1.data );
	mavlink_msg_flash_fw_send_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_flash_fw_send_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_send_block_send(MAVLINK_COMM_1 , packet1.resp , packet1.type , packet1.addr , packet1.length , packet1.data );
	mavlink_msg_flash_fw_send_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_flash_fw_write_block(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FLASH_FW_WRITE_BLOCK >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_flash_fw_write_block_t packet_in = {
		963497464,17443,151
    };
	mavlink_flash_fw_write_block_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.addr = packet_in.addr;
        packet1.length = packet_in.length;
        packet1.resp = packet_in.resp;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_block_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_flash_fw_write_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_block_pack(system_id, component_id, &msg , packet1.resp , packet1.addr , packet1.length );
	mavlink_msg_flash_fw_write_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_block_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp , packet1.addr , packet1.length );
	mavlink_msg_flash_fw_write_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_flash_fw_write_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_write_block_send(MAVLINK_COMM_1 , packet1.resp , packet1.addr , packet1.length );
	mavlink_msg_flash_fw_write_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_flash_fw_erase(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FLASH_FW_ERASE >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_flash_fw_erase_t packet_in = {
		5,{ 72, 73, 74, 75, 76, 77, 78, 79 }
    };
	mavlink_flash_fw_erase_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.resp = packet_in.resp;
        
        mav_array_memcpy(packet1.param, packet_in.param, sizeof(uint8_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_erase_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_flash_fw_erase_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_erase_pack(system_id, component_id, &msg , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_erase_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_erase_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_erase_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_flash_fw_erase_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_flash_fw_erase_send(MAVLINK_COMM_1 , packet1.resp , packet1.param );
	mavlink_msg_flash_fw_erase_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_opencr_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_ack(system_id, component_id, last_msg);
	mavlink_test_read_version(system_id, component_id, last_msg);
	mavlink_test_flash_fw_write_begin(system_id, component_id, last_msg);
	mavlink_test_flash_fw_write_end(system_id, component_id, last_msg);
	mavlink_test_flash_fw_send_block(system_id, component_id, last_msg);
	mavlink_test_flash_fw_write_block(system_id, component_id, last_msg);
	mavlink_test_flash_fw_erase(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // OPENCR_MSG_TESTSUITE_H
