#include "dialog.h"
#include "ui_dialog.h"
#include "qextserialport.h"
#include "qextserialenumerator.h"
#include <QtCore>
#include <QFileDialog>
#include <QTextCodec>
#include <QCoreApplication>
#include <QMessageBox>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QDebug>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sys/time.h>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    //! [0]
    foreach (QextPortInfo info, QextSerialEnumerator::getPorts())
        ui->portBox->addItem(info.portName);
    //make sure user can input their own port name!
    ui->portBox->setEditable(true);

    ui->baudRateBox->addItem("1200", BAUD1200);
    ui->baudRateBox->addItem("2400", BAUD2400);
    ui->baudRateBox->addItem("4800", BAUD4800);
    ui->baudRateBox->addItem("9600", BAUD9600);
    ui->baudRateBox->addItem("19200", BAUD19200);
    ui->baudRateBox->addItem("115200", BAUD115200);
    ui->baudRateBox->setCurrentIndex(5);

    ui->parityBox->addItem("NONE", PAR_NONE);
    ui->parityBox->addItem("ODD", PAR_ODD);
    ui->parityBox->addItem("EVEN", PAR_EVEN);

    ui->dataBitsBox->addItem("5", DATA_5);
    ui->dataBitsBox->addItem("6", DATA_6);
    ui->dataBitsBox->addItem("7", DATA_7);
    ui->dataBitsBox->addItem("8", DATA_8);
    ui->dataBitsBox->setCurrentIndex(3);

    ui->stopBitsBox->addItem("1", STOP_1);
    ui->stopBitsBox->addItem("2", STOP_2);

    ui->queryModeBox->addItem("Polling", QextSerialPort::Polling);
    ui->queryModeBox->addItem("EventDriven", QextSerialPort::EventDriven);
    //! [0]

    ui->led_SerialOnOff->turnOff();
    ui->led_MavlinkStatus->turnOff();
    ui->led_Tx->turnOff();
    ui->led_Rx->turnOff();

    timer = new QTimer(this);
    timer->setInterval(40);

    timer_colck = new QTimer(this);
    connect(timer_colck, SIGNAL(timeout()), this, SLOT(onClockLabelUpdate()));
    timer_colck->start(1000);

    //! [1]
    PortSettings settings = {BAUD9600, DATA_8, PAR_NONE, STOP_1, FLOW_OFF, 10};
    port = new QextSerialPort(ui->portBox->currentText(), settings, QextSerialPort::Polling);
    //! [1]

    enumerator = new QextSerialEnumerator(this);
    enumerator->setUpNotifications();

    connect(ui->baudRateBox, SIGNAL(currentIndexChanged(int)), SLOT(onBaudRateChanged(int)));
    connect(ui->parityBox, SIGNAL(currentIndexChanged(int)), SLOT(onParityChanged(int)));
    connect(ui->dataBitsBox, SIGNAL(currentIndexChanged(int)), SLOT(onDataBitsChanged(int)));
    connect(ui->stopBitsBox, SIGNAL(currentIndexChanged(int)), SLOT(onStopBitsChanged(int)));
    connect(ui->queryModeBox, SIGNAL(currentIndexChanged(int)), SLOT(onQueryModeChanged(int)));
    connect(ui->timeoutBox, SIGNAL(valueChanged(int)), SLOT(onTimeoutChanged(int)));
    connect(ui->portBox, SIGNAL(editTextChanged(QString)), SLOT(onPortNameChanged(QString)));
    connect(ui->openCloseButton, SIGNAL(clicked()), SLOT(onOpenCloseButtonClicked()));

    connect(timer, SIGNAL(timeout()), SLOT(onReadyRead()));
    connect(port, SIGNAL(readyRead()), SLOT(onReadyRead()));

    connect(enumerator, SIGNAL(deviceDiscovered(QextPortInfo)), SLOT(onPortAddedOrRemoved()));
    connect(enumerator, SIGNAL(deviceRemoved(QextPortInfo)), SLOT(onPortAddedOrRemoved()));

    ui->tb_hexview->setRowCount(1);
    ui->tb_hexview->setColumnCount(1);
    ui->tb_hexview->setItem(0, 0, new QTableWidgetItem(" "));

    QDateTime local(QDateTime::currentDateTime());
    ui->label_13->setText(local.toString());

    ui->progressBar_Status->setValue(100);

    setWindowTitle("OpenCR Firmware Downloader v1.0");
}

Dialog::~Dialog()
{
    delete ui;
}
void Dialog::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void Dialog::onPortNameChanged(const QString & /*name*/)
{
    if (port->isOpen()) {
        port->close();
        ui->led_SerialOnOff->turnOff();
    }
}
//! [2]
void Dialog::onBaudRateChanged(int idx)
{
    port->setBaudRate((BaudRateType)ui->baudRateBox->itemData(idx).toInt());
}

void Dialog::onParityChanged(int idx)
{
    port->setParity((ParityType)ui->parityBox->itemData(idx).toInt());
}

void Dialog::onDataBitsChanged(int idx)
{
    port->setDataBits((DataBitsType)ui->dataBitsBox->itemData(idx).toInt());
}

void Dialog::onStopBitsChanged(int idx)
{
    port->setStopBits((StopBitsType)ui->stopBitsBox->itemData(idx).toInt());
}

void Dialog::onQueryModeChanged(int idx)
{
    port->setQueryMode((QextSerialPort::QueryMode)ui->queryModeBox->itemData(idx).toInt());
}

void Dialog::onTimeoutChanged(int val)
{
    port->setTimeout(val);
}
//! [2]
//! [3]
void Dialog::onOpenCloseButtonClicked()
{
    if (!port->isOpen())
    {
        if(ui->portBox->currentText() != NULL )
        {
            port->setPortName(ui->portBox->currentText());
            port->open(QIODevice::ReadWrite);

            onTextBoxLogPrint("Opened "+port->portName()+"\r\n");
        }
        else
        {
            onTextBoxLogPrint("There is no serial device..\r\n");
        }
    }
    else
    {
        port->close();
        onTextBoxLogPrint("Colsed "+port->portName()+"\r\n");
    }

    //If using polling mode, we need a QTimer
    if (port->isOpen() && port->queryMode() == QextSerialPort::Polling)
        timer->start();
    else
        timer->stop();

    //update led's status
    ui->led_SerialOnOff->turnOn(port->isOpen());
}
//! [3]
//! [4]

void Dialog::onReadyRead()
{
    /*if (port->bytesAvailable()) {
        QByteArray ba = port->readAll();
        ui->textEdit_Log->moveCursor(QTextCursor::End);
        ui->textEdit_Log->insertPlainText(QString::fromLatin1(ba));
    }*/

    receiveFlag = true;
}

void Dialog::onClockLabelUpdate()
{
    QDateTime local(QDateTime::currentDateTime());
    ui->label_13->setText(local.toString("hh:mm:ss A"));
    if(ui->led_Rx->ledStatus())ui->led_Rx->turnOff();
    if(ui->led_Tx->ledStatus())ui->led_Tx->turnOff();

    if(!ui->led_SerialOnOff->ledStatus())ui->led_MavlinkStatus->turnOff();
    //onTextBoxLogPrint("test\r\n");
}

void Dialog::onPortAddedOrRemoved()
{
    QString current = ui->portBox->currentText();

    ui->portBox->blockSignals(true);
    ui->portBox->clear();
    foreach (QextPortInfo info, QextSerialEnumerator::getPorts())
        ui->portBox->addItem(info.portName);

    ui->portBox->setCurrentIndex(ui->portBox->findText(current));

    ui->portBox->blockSignals(false);
}

void Dialog::on_LoadFirmwareButton_clicked()
{
    QFileDialog fileDialog(this,tr("Open File"),
            QCoreApplication::applicationDirPath(),
            tr("Img(*.bin)"));

    //fileNames = NULL;
    if(fileDialog.exec()){
        fileNames = fileDialog.selectedFiles();
    }

    QString selectedFile;
    for(int nIndex=0; nIndex < fileNames.size(); nIndex++){
        selectedFile.append(fileNames.at(nIndex).toLocal8Bit().constData());
    }

    //QTextCodec *textCodec = QTextCodec::codecForName("eucKR");
   // ui->textEdit_Log->setText(selectedFile.toUtf8());
    onTextBoxLogPrint(selectedFile.toUtf8()+"\r\n");

    QFile file(selectedFile.toUtf8());
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::information(0,"error",file.errorString());
        return;
    }

    //QDataStream dataStreamReader(&file);
    //dataStreamReader >> ba;

   // ba = file.readAll();

    //QString s2 = ba.toHex();
    //qDebug() << s2;
    //onTextBoxLogPrint(s2);
       // ui->progressBar_Status->setValue(100*line.count()*line.size());
   // }

    QString textPrint;
    int file_size = file.size();
    onTextBoxLogPrint(textPrint.sprintf("file size : %d bytes\r\n",file_size));
    ui->tb_hexview->setRowCount((file_size/16) +1);
    ui->tb_hexview->setColumnCount(4);

    for(int index=0; index < file_size; index = index+4)
    {
        QByteArray ba;
        QString tmpstr;

        char buf[4];
        file.seek(index);
        file.read(buf,4);

        ba = QByteArray::fromRawData(buf, 4);

        tmpstr = ba.mid(3,1).toHex().rightJustified(2,'0');
        tmpstr += ba.mid(2,1).toHex().rightJustified(2,'0');
        tmpstr += ba.mid(1,1).toHex().rightJustified(2,'0');
        tmpstr += ba.mid(0,1).toHex().rightJustified(2,'0');



        //tmpstr.append( QString::number(ba.at(3), 16).rightJustified(2, '0') );
        //tmpstr.append( QString::number(ba.at(2), 16).rightJustified(2, '0') );
        //tmpstr.append( QString::number(ba.at(1), 16).rightJustified(2, '0') );
        //tmpstr.append( QString::number(ba.at(0), 16).rightJustified(2, '0') );

        //if(49152<index)
        {
            //qDebug()<< index << tmpstr;
            //if (port->isOpen())
            {
               // QString textPrint;
               // QString strportName = textPrint.sprintf("%d : ",index) + tmpstr;
               // QByteArray strba = strportName.toLatin1();
                //char * p_data = ba.data();
                //port->write(p_data,ba.length());
               // port->write(strba);

      //          port->write(tmpstr.toLatin1()+"\r\n");
            }
               // port->write("test");
               //port->write(ui->textEdit_Log->toPlainText().toLatin1());

        }
        ui->tb_hexview->setItem((index/4)/4, (index/4)%4, new QTableWidgetItem(tmpstr.toUpper()));

        ui->progressBar_Status->setValue(100*(index+4)/file_size);

    }

    file.close();
}


void Dialog::on_ProgramButton_clicked()
{
    QFutureWatcher<void> watcher;
    QObject::connect(&watcher, SIGNAL(finished()), SLOT(ProgramThread_quit()));

    QFuture<void> future = QtConcurrent::run(this,ProgramThread_run);
    watcher.setFuture(future);

  return;
}
void Dialog::ProgramThread_quit()
{

    return;
}
void Dialog::ProgramThread_run()
{
    int i;
    int j;
    int ret = 0;
    err_code_t err_code = OK;
    long t, dt;
    float calc_time;
    uint32_t fw_size = 256*1024*3;
    uint8_t  board_str[16];
    uint8_t  board_str_len;
    uint32_t board_version;
    uint32_t board_revision;
    uint32_t crc;
    uint32_t crc_ret = 0;
    uint8_t  *p_buf_crc;
    char *portname;
    char *filename;
    uint32_t baud;
    uint8_t  block_buf[FLASH_TX_BLOCK_LENGTH];
    uint32_t addr;
    uint32_t len;

    baud     = (uint32_t)port->baudRate();
    //QString --> QByteArray --> const char*

    QString strportName = port->portName();
    QByteArray ba = strportName.toLatin1();
    portname = ba.data();

    // Open port
    if (!port->isOpen())
    {
        onTextBoxLogPrint("serial port is not opened\r\n");
        return;
    }

    QString selectedFile;
    for(int nIndex=0; nIndex < fileNames.size(); nIndex++)
    {
        selectedFile.append(fileNames.at(nIndex).toLocal8Bit().constData());
    }
    QByteArray bafilename = selectedFile.toLatin1();
    filename = bafilename.data();

    QString textPrint;
    if( ( opencr_fp = fopen(filename, "rb" ) ) == NULL )
    {
        onTextBoxLogPrint(textPrint.sprintf("Failed to open file : %s\r\n",filename));
        return;
        //exit( 1 );
    }
    else
    {
        fseek( opencr_fp, 0, SEEK_END );
        opencr_fpsize = ftell( opencr_fp );
        fseek( opencr_fp, 0, SEEK_SET );
        onTextBoxLogPrint(">>\r\n");
        onTextBoxLogPrint(textPrint.sprintf("file name :\r\n %s \r\n", filename));
        onTextBoxLogPrint(textPrint.sprintf("file size : %d KB\r\n", opencr_fpsize/1024));
    }

  fw_size = opencr_fpsize;

  t = iclock();
  ret = opencr_ld_flash_erase(fw_size);
  dt = iclock() - t;

  onTextBoxLogPrint(textPrint.sprintf("flash_erase : %d : %f sec\r\n", ret, GET_CALC_TIME(dt)));
  if( ret < 0 )
  {
    //port->close();
    fclose( opencr_fp );
    onTextBoxLogPrint("erase flash failed...\r\n");
    return;//exit(1);
  }
  else
  {
      onTextBoxLogPrint("erase flash completed...\r\n");
  }

  t = iclock();
  crc  = 0;
  addr = 0;
  while(1)
  {
    // ui->progressBar_Status->setValue(100*addr/fw_size);
     len = opencr_ld_file_read_data( block_buf, FLASH_TX_BLOCK_LENGTH);
    if( len == 0 ) break;

    for( i=0; i<len; i++ )
    {
      crc = crc_calc( crc,  block_buf[i] );
    }

    ret = opencr_ld_flash_write( addr, block_buf, len );
    if( ret < 0 ) break;
    addr += len;

    ui->textEdit_Log->insertPlainText(".");
  }
  ui->textEdit_Log->insertPlainText("\r\n");
  dt = iclock() - t;

 onTextBoxLogPrint(textPrint.sprintf("flash_write : %d : %f sec \r\n", ret,  GET_CALC_TIME(dt)));
  if( ret < 0 )
  {
    port->close();
    fclose( opencr_fp );
    return;
  }

  t = iclock();
  err_code = cmd_flash_fw_verify( fw_size, crc, &crc_ret );
  dt = iclock() - t;
  if( err_code == OK )
  {
    onTextBoxLogPrint(textPrint.sprintf("CRC OK %X %X %f sec\r\n", crc, crc_ret, GET_CALC_TIME(dt)));
  }
  else
  {
    onTextBoxLogPrint(textPrint.sprintf("CRC Fail : 0x%X : %X, %X %f sec\r\n", err_code, crc, crc_ret, GET_CALC_TIME(dt)));
  }

  onTextBoxLogPrint(textPrint.sprintf("jump_to_fw \r\n"));
  cmd_jump_to_fw();

  port->close();
  fclose( opencr_fp );
}




//==================================================================
//member functions

void Dialog::onTextBoxLogPrint(QString str)
{
    QDateTime local(QDateTime::currentDateTime());

    ui->textEdit_Log->setTextColor(QColor(0x00,0x00,0x97));
    ui->textEdit_Log->insertPlainText("-"+local.toString("hh:mm:ss A")+" : ");

//ui->textEdit_Log->textCursor().deletePreviousChar();
    ui->textEdit_Log->moveCursor (QTextCursor::End);
    ui->textEdit_Log->setTextColor(QColor(0x00,0x00,0x00));
    ui->textEdit_Log->insertPlainText(str);
    ui->textEdit_Log->moveCursor (QTextCursor::End);


  //  ui->textEdit_Log->append(str);

}

/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_file_read_data
     WORK    :
---------------------------------------------------------------------------*/
uint32_t Dialog::opencr_ld_file_read_data( uint8_t *dst, uint32_t len )
{
  size_t readbytes = 0;

  if( !feof( opencr_fp ) )
  {
    readbytes = fread( dst, 1, len, opencr_fp );
  }
  return ( uint32_t )readbytes;
}

/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_write
     WORK    :
---------------------------------------------------------------------------*/
int Dialog::opencr_ld_flash_write( uint32_t addr, uint8_t *p_data, uint32_t length  )
{
  int ret = 0;
  err_code_t err_code = OK;
  uint32_t block_length;
  uint16_t block_cnt;
  uint32_t written_packet_length;
  uint32_t written_total_length;
  uint32_t packet_length = 128;
  uint32_t i;

   QString textPrint;

  err_code = cmd_flash_fw_write_begin();
  if( err_code != OK )
  {
    onTextBoxLogPrint(textPrint.sprintf("cmd_flash_fw_write_begin ERR : 0x%04X\r\n", err_code));
    return -1;
  }

  written_total_length = 0;

  while(1)
  {
    block_length = length - written_total_length;

    if( block_length > FLASH_TX_BLOCK_LENGTH )
    {
      block_length = FLASH_TX_BLOCK_LENGTH;
    }

    block_cnt = block_length/FLASH_PACKET_LENGTH;
    if( block_length%FLASH_PACKET_LENGTH > 0 )
    {
      block_cnt += 1;
    }


    written_packet_length = 0;
    for( i=0; i<block_cnt; i++ )
    {
      packet_length = block_length - written_packet_length;
      if( packet_length > FLASH_PACKET_LENGTH )
      {
    packet_length = FLASH_PACKET_LENGTH;
      }

      err_code = cmd_flash_fw_write_packet(written_packet_length, &p_data[written_total_length+written_packet_length], packet_length);
      if( err_code != OK )
      {
    onTextBoxLogPrint(textPrint.sprintf("cmd_flash_fw_send_block ERR : 0x%04X\r\n", err_code));
    return -2;
      }

      written_packet_length += packet_length;
    }

    //onTextBoxLogPrint(textPrint.sprintf("%d : %d, %d, %d \r\n", written_packet_length, block_length, block_cnt, packet_length));

    if( written_packet_length == block_length )
    {
      err_code = cmd_flash_fw_write_block(addr+written_total_length, block_length);
      if( err_code != OK )
      {
    onTextBoxLogPrint(textPrint.sprintf("cmd_flash_fw_write_block ERR : 0x%04X\r\n", err_code));
    return -3;
      }
    }
    else
    {
      onTextBoxLogPrint(textPrint.sprintf("written_packet_length : %d, %d 0x%04X\r\n", written_packet_length, block_length, err_code));
      return -4;
    }

    written_total_length += block_length;

    if( written_total_length == length )
    {
      break;
    }
    else if( written_total_length > length )
    {
      onTextBoxLogPrint(textPrint.sprintf("written_total_length over \r\n"));
      return -5;
    }
  }


  cmd_flash_fw_write_end();

  return ret;
}

/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_read
     WORK    :
---------------------------------------------------------------------------*/
int Dialog::opencr_ld_flash_read( uint32_t addr, uint8_t *p_data, uint32_t length  )
{
  int ret = 0;
  err_code_t err_code = OK;
  uint32_t block_length;
  uint32_t read_packet_length;
  uint32_t read_total_length;
  int i;
  int err_count = 0;

  QString textPrint;

  read_total_length = 0;

  while(1)
  {
    block_length = length - read_total_length;

    if( block_length > FLASH_PACKET_LENGTH )
    {
      block_length = FLASH_PACKET_LENGTH;
    }


    for( i=0; i<3; i++ )
    {
      err_code = cmd_flash_fw_read_block( addr+read_total_length, &p_data[read_total_length], block_length );
      if( err_code == OK ) break;
      err_count++;
    }


    if( err_code != OK )
    {
      onTextBoxLogPrint(textPrint.sprintf("cmd_flash_fw_read_block : addr:%X, 0x%04X \r\n", addr+read_total_length, err_code));
      return -1;
    }

    read_total_length += block_length;

    if( read_total_length == length )
    {
      break;
    }
    else if( read_total_length > length )
    {
      onTextBoxLogPrint(textPrint.sprintf("read_total_length over \r\n"));
      return -2;
    }
  }

  return ret;
}

/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_erase
     WORK    :
---------------------------------------------------------------------------*/
int Dialog::opencr_ld_flash_erase( uint32_t length  )
{
  int ret = 0;
  err_code_t err_code = OK;

  QString textPrint;
  err_code = cmd_flash_fw_erase( length );

  if( err_code != OK )
  {
    onTextBoxLogPrint(textPrint.sprintf("cmd_flash_fw_erase_block : 0x%04X %d\r\n", err_code, length) );
    return -1;
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_version
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_read_version( uint32_t *p_version, uint32_t *p_revision )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_read_version_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      //onTextBoxLogPrint(textPrint.sprintf("BootVersion : 0x%08X\r\n", ack_msg.data[3]<<24|ack_msg.data[2]<<16|ack_msg.data[1]<<8|ack_msg.data[0]));
      *p_version  = ack_msg.data[3]<<24|ack_msg.data[2]<<16|ack_msg.data[1]<<8|ack_msg.data[0];
      *p_revision = ack_msg.data[7]<<24|ack_msg.data[6]<<16|ack_msg.data[5]<<8|ack_msg.data[4];
      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_board_name
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_read_board_name( uint8_t *p_str, uint8_t *p_len )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_read_board_name_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      *p_len = ack_msg.length;
      memcpy(p_str, ack_msg.data, ack_msg.length);
      p_str[ack_msg.length] = 0;

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_erase
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_flash_fw_erase( uint32_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_erase_pack(0, 0, &tx_msg, resp, length, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 10000) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}

/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_begin
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_flash_fw_write_begin( void )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_write_begin_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_end
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_flash_fw_write_end( void )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_write_end_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      //onTextBoxLogPrint(textPrint.sprintf("block_count  : %d\r\n", ack_msg.data[1]<<8|ack_msg.data[0]));
      //onTextBoxLogPrint(textPrint.sprintf("block_length : %d\r\n", ack_msg.data[5]<<24|ack_msg.data[4]<<16|ack_msg.data[3]<<8|ack_msg.data[2]));


      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_packet
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_flash_fw_write_packet( uint16_t addr, uint8_t *p_data, uint8_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t resp = 0;



  mavlink_msg_flash_fw_write_packet_pack(0, 0, &tx_msg, resp, addr, length, p_data);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_send_block_multi
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_flash_fw_send_block_multi( uint8_t block_count )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t buf[256];
  uint8_t tx_buf[16*1024];
  uint8_t resp = 0;
  uint8_t i;
  uint32_t len;


   len = 0;
  for( i=0; i<block_count; i++ )
  {
    mavlink_msg_flash_fw_write_packet_pack(0, 0, &tx_msg, resp, 0, 128, buf);
    len += mavlink_msg_to_send_buffer(&tx_buf[len], &tx_msg);
  }
  write_bytes((char *)tx_buf, len);

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_block
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_flash_fw_write_block( uint32_t addr, uint32_t length  )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t buf[256];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_write_block_pack(0, 0, &tx_msg, resp, addr, length);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_read_block
     WORK    :
---------------------------------------------------------------------------*/
#if 0
err_code_t Dialog::cmd_flash_fw_read_block( uint32_t addr, uint8_t *p_data, uint16_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_flash_fw_read_t  resp_msg;
  uint8_t resp = 1;
  uint16_t received_length;


  mavlink_msg_flash_fw_read_block_pack(0, 0, &tx_msg, resp, addr, length);
  msg_send(0, &tx_msg);



  if( resp == 1 )
  {
    received_length = 0;

    while(1)
    {
      if( msg_get_resp(0, &rx_msg, 3000) == TRUE )
      {
    mavlink_msg_flash_fw_read_decode( &rx_msg, &resp_msg);

    memcpy(&p_data[received_length], resp_msg.data, resp_msg.length);
    received_length += resp_msg.length;

    //onTextBoxLogPrint(textPrint.sprintf("recv %d \r\n", received_length));

    if( received_length == length )
    {
      break;
    }
    else if( received_length > length )
    {
      err_code = ERR_SIZE_OVER;
      break;
    }
      }
      else
      {
    err_code = ERR_TIMEOUT;
    break;
      }
    }
  }

  return err_code;
}
#else
err_code_t Dialog::cmd_flash_fw_read_block( uint32_t addr, uint8_t *p_data, uint16_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_flash_fw_read_packet_t  resp_msg;
  uint8_t resp = 1;


  mavlink_msg_flash_fw_read_block_pack(0, 0, &tx_msg, resp, addr, length);
  msg_send(0, &tx_msg);



  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 100) == TRUE )
    {
      mavlink_msg_flash_fw_read_packet_decode( &rx_msg, &resp_msg);

      memcpy(p_data, resp_msg.data, resp_msg.length);

      if( resp_msg.length > length )
      {
    err_code = ERR_SIZE_OVER;
      }
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}
#endif


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_verify
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_flash_fw_verify( uint32_t length, uint32_t crc, uint32_t *p_crc_ret )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_verify_pack(0, 0, &tx_msg, resp, length, crc, param);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      *p_crc_ret = ack_msg.data[3]<<24|ack_msg.data[2]<<16|ack_msg.data[1]<<8|ack_msg.data[0];

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }
  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_jump_to_fw
     WORK    :
---------------------------------------------------------------------------*/
err_code_t Dialog::cmd_jump_to_fw(void)
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 0;


  mavlink_msg_jump_to_fw_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }
  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : crc_calc
     WORK    :
---------------------------------------------------------------------------*/
uint32_t Dialog::crc_calc( uint32_t crc_in, uint8_t data_in )
{

  crc_in  ^= data_in;
  crc_in  += data_in;

  return crc_in;
}




void Dialog::msg_send(uint8_t chan, mavlink_message_t *p_msg)
{
  uint8_t  buf[1024];
  uint16_t len;
  uint16_t write_len;
QString textPrint;

ui->led_Tx->turnOn();
  len = mavlink_msg_to_send_buffer(buf, p_msg);

  switch(chan)
  {
    case 0:
      write_len = write_bytes((char *)buf, (uint32_t)len);
      if( write_len != len ) onTextBoxLogPrint(textPrint.sprintf("wlen %d : len %d\r\n", write_len, len));
      break;

    case 1:
      break;
  }

  //ui->led_Tx->turnOff();
}


BOOL Dialog::msg_recv( uint8_t chan, uint8_t data , mavlink_message_t *p_msg, mavlink_status_t *p_status )
{
  BOOL ret = FALSE;
    ui->led_MavlinkStatus->turnOn();
  if(chan == 0)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, data, p_msg, p_status) == MAVLINK_FRAMING_OK)
    {
      ret = TRUE;
    }
  }
  else
  {
    if (mavlink_parse_char(MAVLINK_COMM_1, data, p_msg, p_status) == MAVLINK_FRAMING_OK)
    {
      ret = TRUE;
    }
  }

  return ret;
}


BOOL Dialog::msg_get_resp( uint8_t chan, mavlink_message_t *p_msg, uint32_t timeout)
{
  BOOL ret = FALSE;
  int  ch_ret;
  uint8_t ch;
  static mavlink_message_t msg[MSG_CH_MAX];
  static mavlink_status_t status[MSG_CH_MAX];
  uint32_t retry = timeout;
    int  data_cnt;
 int  time_out_cnt;
    ui->led_Rx->turnOn();
    port->waitForReadyRead(timeout);

    if(receiveFlag == true && port->bytesAvailable())
    {
        QByteArray ch_ret = port->readAll();
        for(data_cnt=0;data_cnt< ch_ret.length();data_cnt++)
        {
           ch = (int)ch_ret.at(data_cnt) ;

            ret = msg_recv( chan, ch, &msg[chan], &status[chan] );

            if( ret == TRUE )
            {
                *p_msg = msg[chan];
                break;
            }
        }
    }

    for(time_out_cnt=0;time_out_cnt<10;time_out_cnt++)
    {
        if( ret != TRUE)
        {
           // port->waitForReadyRead(10000);
            QThread::sleep(1);
            if(port->bytesAvailable())
            {
                QByteArray ch_ret = port->readAll();
                for(data_cnt=0;data_cnt< ch_ret.length();data_cnt++)
                {
                   ch = (int)ch_ret.at(data_cnt) ;

                    ret = msg_recv( chan, ch, &msg[chan], &status[chan] );

                    if( ret == TRUE )
                    {
                        *p_msg = msg[chan];
                        break;
                    }
                }
            }
       }
       else
       {
            break;
       }
    }
    receiveFlag =false;

   // ui->led_Rx->g->turnOff();
    return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : read_byte
     WORK    :
---------------------------------------------------------------------------*/
void Dialog::ser_set_timeout_ms(long val )
{
    port->setTimeout(val);

    return;
}

/*---------------------------------------------------------------------------
     TITLE   : read_byte
     WORK    :
---------------------------------------------------------------------------*/
int Dialog::read_byte( void )
{
    //return ser_read_byte( stm32_ser_id );
    char byte;

    port->read(&byte,1);

    return (int)byte;
}


/*---------------------------------------------------------------------------
     TITLE   : write_bytes
     WORK    :
---------------------------------------------------------------------------*/
int Dialog::write_bytes( char *p_data, int len )
{
    int written_len;

    //written_len = ser_write( stm32_ser_id, (const u8 *)p_data, len );
    written_len = port->write(p_data,len);

    return written_len;
}

long Dialog::iclock()
{
   // struct timeval tv;
   // gettimeofday (&tv, NULL);
   // return (tv.tv_sec * 1000 + tv.tv_usec / 1000);

    QDateTime local(QDateTime::currentDateTime());
    return local.toMSecsSinceEpoch();

}


void Dialog::on_bn_ReadBoardName_clicked()
{
    err_code_t err_code = OK;
    QString textPrint;
    uint8_t  board_str[16];
    uint8_t  board_str_len;

    if(port->isOpen())
    {
        err_code = cmd_read_board_name( board_str, &board_str_len );
        if( err_code == OK )
        {
            onTextBoxLogPrint(textPrint.sprintf("Board Name : %s\r\n", board_str));
        }
    }
    else
    {
        onTextBoxLogPrint("port is not opened...\r\n");
    }
}

void Dialog::on_bn_ReadBoardVersion_clicked()
{
    err_code_t err_code = OK;
    QString textPrint;
    uint32_t board_version;
    uint32_t board_revision;

    if(port->isOpen())
    {
        err_code = cmd_read_version( &board_version, &board_revision );
        if( err_code == OK )
        {
          onTextBoxLogPrint(textPrint.sprintf("Board Ver  : 0x%08X\r\n", board_version));
          onTextBoxLogPrint(textPrint.sprintf("Board Rev  : 0x%08X\r\n", board_revision));
        }


    }
    else
    {
        onTextBoxLogPrint("port is not opened...\r\n");
    }
}

void Dialog::on_sendButton_3_clicked()
{

}
