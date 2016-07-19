#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "./msg/def.h"

#define GET_CALC_TIME(x)	( (int)(x / 1000) + ((float)(x % 1000))/1000 )

#define FLASH_TX_BLOCK_LENGTH	(8*1024)
#define FLASH_RX_BLOCK_LENGTH	(128)
#define FLASH_PACKET_LENGTH   	128

#define MSG_CH_MAX	1

typedef struct
{
  uint8_t ch;
  mavlink_message_t *p_msg;
} msg_t;

namespace Ui {
class Dialog;
}
class QTimer;
class QextSerialPort;
class QextSerialEnumerator;

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
protected:
    void changeEvent(QEvent *e);

private slots:

    void onPortNameChanged(const QString &name);
    void onBaudRateChanged(int idx);
    void onParityChanged(int idx);
    void onDataBitsChanged(int idx);
    void onStopBitsChanged(int idx);
    void onQueryModeChanged(int idx);
    void onTimeoutChanged(int val);
    void onOpenCloseButtonClicked();

    void onReadyRead();
    void onClockLabelUpdate();
    void onPortAddedOrRemoved();

    void on_ProgramButton_clicked();
    void ProgramThread_run();
    void ProgramThread_quit();
    void on_LoadFirmwareButton_clicked();

    void on_bn_ReadBoardName_clicked();

    void on_bn_ReadBoardVersion_clicked();

    void on_sendButton_3_clicked();

private:
    Ui::Dialog *ui;
    QTimer *timer;
    QTimer *timer_colck;
    QextSerialPort *port;
    QextSerialEnumerator *enumerator;

public:
    QStringList fileNames;
     FILE      *opencr_fp;
     uint32_t   opencr_fpsize;

     bool receiveFlag;

    void onTextBoxLogPrint(QString str);
    //int opencr_ld_down( int argc, const char **argv );
    int opencr_ld_flash_write( uint32_t addr, uint8_t *p_data, uint32_t length  );
    int opencr_ld_flash_read( uint32_t addr, uint8_t *p_data, uint32_t length  );
    int opencr_ld_flash_erase( uint32_t length  );
    uint32_t opencr_ld_file_read_data( uint8_t *dst, uint32_t len );

    err_code_t cmd_read_version( uint32_t *p_version, uint32_t *p_revision );
    err_code_t cmd_read_board_name( uint8_t *p_str, uint8_t *p_len );
    err_code_t cmd_flash_fw_erase( uint32_t length );
    err_code_t cmd_flash_fw_write_begin( void );
    err_code_t cmd_flash_fw_write_end( void );
    err_code_t cmd_flash_fw_write_packet( uint16_t addr, uint8_t *p_data, uint8_t length );
    err_code_t cmd_flash_fw_write_block( uint32_t addr, uint32_t length  );
    err_code_t cmd_flash_fw_send_block_multi( uint8_t block_count );
    err_code_t cmd_flash_fw_read_block( uint32_t addr, uint8_t *p_data, uint16_t length );
    err_code_t cmd_flash_fw_verify( uint32_t length, uint32_t crc, uint32_t *p_crc_ret );
    err_code_t cmd_jump_to_fw(void);

    void msg_send(uint8_t chan, mavlink_message_t *p_msg);
    BOOL msg_recv( uint8_t chan, uint8_t data , mavlink_message_t *p_msg, mavlink_status_t *p_status );
    BOOL msg_get_resp( uint8_t chan, mavlink_message_t *p_msg, uint32_t timeout);

    void ser_set_timeout_ms(long val );
    int read_byte( void );
    int write_bytes( char *p_data, int len );

    uint32_t crc_calc( uint32_t crc_in, uint8_t data_in );
    long iclock();
};





#endif // DIALOG_H
