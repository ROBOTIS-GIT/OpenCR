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




static void mavlink_test_read_boot_version(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_READ_BOOT_VERSION >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_read_boot_version_t packet_in = {
		5,{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87 }
    };
	mavlink_read_boot_version_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ver_length = packet_in.ver_length;
        
        mav_array_memcpy(packet1.ver_stirng, packet_in.ver_stirng, sizeof(uint8_t)*16);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_boot_version_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_read_boot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_boot_version_decode(&msg, &packet2);
	mavlink_msg_read_boot_version_pack(system_id, component_id, &msg , packet1.ver_length , packet1.ver_stirng );
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_boot_version_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ver_length , packet1.ver_stirng );
	mavlink_msg_read_boot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_read_boot_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_read_boot_version_send(MAVLINK_COMM_1 , packet1.ver_length , packet1.ver_stirng );
	mavlink_msg_read_boot_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_opencr_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_read_boot_version(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // OPENCR_MSG_TESTSUITE_H

