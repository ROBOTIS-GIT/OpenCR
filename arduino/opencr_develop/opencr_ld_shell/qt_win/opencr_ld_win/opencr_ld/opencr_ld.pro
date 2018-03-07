TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += WIN32_BUILD

SOURCES += \
    ../../../msg/msg.c \
    ../../../main.c \
    ../../../opencr_ld.c \
    ../../../serial_win32.c

HEADERS += \
    ../../../opencr_ld.h \
    ../../../serial.h \
    ../../../type.h \
    ../../../msg/mavlink/opencr_msg/mavlink.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_ack.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_erase.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_read_block.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_read_packet.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_verify.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_begin.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_block.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_end.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_packet.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_jump_to_fw.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_read_board_name.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_read_tag.h \
    ../../../msg/mavlink/opencr_msg/mavlink_msg_read_version.h \
    ../../../msg/mavlink/opencr_msg/opencr_msg.h \
    ../../../msg/mavlink/opencr_msg/testsuite.h \
    ../../../msg/mavlink/opencr_msg/version.h \
    ../../../msg/mavlink/checksum.h \
    ../../../msg/mavlink/mavlink_conversions.h \
    ../../../msg/mavlink/mavlink_helpers.h \
    ../../../msg/mavlink/mavlink_types.h \
    ../../../msg/mavlink/protocol.h \
    ../../../msg/def.h \
    ../../../msg/def_err.h \
    ../../../msg/msg.h
