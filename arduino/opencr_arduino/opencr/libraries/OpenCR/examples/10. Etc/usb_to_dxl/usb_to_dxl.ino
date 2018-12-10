#include <DynamixelSDK.h>
#include <stdarg.h>


#define DXL_USB_VER           20170915

#define CMD_PORT              Serial      // USB
#define DBG_PORT              Serial2     // UART1
#define DXL_PORT              Serial3
#define DXL_BAUD              1000000


#define DXL_LED_RX            BDPIN_LED_USER_1
#define DXL_LED_TX            BDPIN_LED_USER_2


#define DXL_POWER_DISABLE()   digitalWrite(BDPIN_DXL_PWR_EN, LOW);
#define DXL_POWER_ENABLE()    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);

#define DXL_TX_BUFFER_LENGTH  1024


uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH];


static int rx_led_count = 0;
static int tx_led_count = 0;

static int rx_led_update_time;
static int tx_led_update_time;

static uint32_t update_time[8];


static uint32_t rx_data_cnt = 0;
static uint32_t tx_data_cnt = 0;

static uint32_t rx_bandwidth = 0;
static uint32_t tx_bandwidth = 0;

uint32_t usb_baud;


void setup()
{
  CMD_PORT.begin(115200);
  DBG_PORT.begin(57600);
  DXL_PORT.begin(DXL_BAUD);

  pinMode( BDPIN_DXL_PWR_EN, OUTPUT );
  pinMode( DXL_LED_RX, OUTPUT );
  pinMode( DXL_LED_TX, OUTPUT );

  digitalWrite(DXL_LED_TX, HIGH);
  digitalWrite(DXL_LED_RX, HIGH);

  drv_dxl_tx_enable(FALSE);

  DXL_POWER_ENABLE();
}

void loop()
{
  update_dxl();
  update_led();

  if( CMD_PORT.getBaudRate() != DXL_PORT.getBaudRate() )
  {
    DXL_PORT.begin(CMD_PORT.getBaudRate());
  }

  if( (millis()-update_time[1]) > 1000 )
  {
    update_time[1] = millis();

    tx_bandwidth = tx_data_cnt;
    rx_bandwidth = rx_data_cnt;

    tx_data_cnt = 0;
    rx_data_cnt = 0;
  }
}


void update_dxl()
{
  int length;
  int i;


  //-- USB -> DXL
  length = CMD_PORT.available();
  if( length > 0 )
  {
    drv_dxl_tx_enable(TRUE);
    for(i=0; i<length; i++ )
    {
      DXL_PORT.write(CMD_PORT.read());
      DXL_PORT.flush();
    }
    drv_dxl_tx_enable(FALSE);

    tx_led_count = 3;

    tx_data_cnt += length;
  }

  //-- DXL -> USB
  length = DXL_PORT.available();
  if( length > 0 )
  {
    if( length > DXL_TX_BUFFER_LENGTH )
    {
      length = DXL_TX_BUFFER_LENGTH;
    }
    for(i=0; i<length; i++ )
    {
      tx_buffer[i] = DXL_PORT.read();
    }
    CMD_PORT.write(tx_buffer, length);

    rx_led_count = 3;
    rx_data_cnt += length;
  }
}


void update_led()
{
  if( (millis()-tx_led_update_time) > 50 )
  {
    tx_led_update_time = millis();

    if( tx_led_count )
    {
      digitalWrite(DXL_LED_TX, !digitalRead(DXL_LED_TX));
      tx_led_count--;
    }
    else
    {
      digitalWrite(DXL_LED_TX, HIGH);
    }
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if( rx_led_count )
    {
      digitalWrite(DXL_LED_RX, !digitalRead(DXL_LED_RX));
      rx_led_count--;
    }
    else
    {
      digitalWrite(DXL_LED_RX, HIGH);
    }
  }
}
