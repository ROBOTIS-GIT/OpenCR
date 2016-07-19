#-------------------------------------------------
#
# Project created by QtCreator 2016-06-04T08:42:06
#
#-------------------------------------------------

QT += core gui
QT += concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = opencr_ld_gui
TEMPLATE = app

include(./qextserialport/qextserialport.pri)

SOURCES += main.cpp\
        dialog.cpp \
    hled.cpp

HEADERS  += dialog.h \
    hled.h \
    msg/mavlink/opencr_msg/mavlink.h \
    msg/mavlink/opencr_msg/mavlink_msg_ack.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_erase.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_read_block.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_read_packet.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_verify.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_begin.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_block.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_end.h \
    msg/mavlink/opencr_msg/mavlink_msg_flash_fw_write_packet.h \
    msg/mavlink/opencr_msg/mavlink_msg_jump_to_fw.h \
    msg/mavlink/opencr_msg/mavlink_msg_read_board_name.h \
    msg/mavlink/opencr_msg/mavlink_msg_read_tag.h \
    msg/mavlink/opencr_msg/mavlink_msg_read_version.h \
    msg/mavlink/opencr_msg/opencr_msg.h \
    msg/mavlink/opencr_msg/testsuite.h \
    msg/mavlink/opencr_msg/version.h \
    msg/mavlink/checksum.h \
    msg/mavlink/mavlink_conversions.h \
    msg/mavlink/mavlink_helpers.h \
    msg/mavlink/mavlink_types.h \
    msg/mavlink/protocol.h \
    msg/def.h \
    msg/def_err.h \
    type.h


FORMS    += dialog.ui

