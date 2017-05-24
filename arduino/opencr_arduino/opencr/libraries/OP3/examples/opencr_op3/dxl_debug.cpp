/*
 *  dxl_node_op3.cpp
 *
 *  debug node op3
 *
 *  Created on: 2017. 2. 2.
 *      Author: Baram
 */

#include "dxl_hw.h"
#include "dxl_hw_op3.h"
#include "dxl_node_op3.h"
#include "dxl_debug.h"
#include <EEPROM.h>



#define DEBUG_SERIAL    Serial




extern dxl_mem_op3_t *p_dxl_mem;

static uint8_t debug_state = 0;



static void dxl_debug_menu_show_list(void);
static bool dxl_debug_menu_loop(uint8_t ch);
static void dxl_debug_menu_show_cmdline(void);
static bool dxl_debug_menu_shwo_ctrltbl();



/*---------------------------------------------------------------------------
     TITLE   : dxl_debug_init
     WORK    :
---------------------------------------------------------------------------*/
void dxl_debug_init(void)
{
  DEBUG_SERIAL.begin(57600);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_debug_loop
     WORK    :
---------------------------------------------------------------------------*/
void dxl_debug_loop(void)
{
  static uint32_t tTime[16];
  uint8_t ch;


  if(Serial.available())
  {
    ch = Serial.read();


    switch(debug_state)
    {
      case 0:
        if(ch == 'm')
        {
          debug_state = 1;
          dxl_debug_menu_show_list();
          dxl_debug_menu_show_cmdline();
        }
        break;

      case 1:
        dxl_debug_menu_loop(ch);
        break;

      default:
        break;
    }
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_debug_menu_show_list
     WORK    :
---------------------------------------------------------------------------*/
void dxl_debug_menu_show_list(void)
{
  DEBUG_SERIAL.println("---------------------------");
  DEBUG_SERIAL.println("m - show menu");
  DEBUG_SERIAL.println("d - show step");
  DEBUG_SERIAL.println("l - show control table");
  DEBUG_SERIAL.println("q - exit menu");
  DEBUG_SERIAL.println("---------------------------");
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_debug_menu_show_cmdline
     WORK    :
---------------------------------------------------------------------------*/
void dxl_debug_menu_show_cmdline(void)
{
  DEBUG_SERIAL.print(">>");
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_debug_menu_show_list
     WORK    :
---------------------------------------------------------------------------*/
bool dxl_debug_menu_loop(uint8_t ch)
{
  bool exit_menu = false;


  switch(ch)
  {
    case 'm':
      dxl_debug_menu_show_list();
      break;


    case 'q':
      exit_menu = true;
      DEBUG_SERIAL.println(" ");
      DEBUG_SERIAL.println("exit menu...");
      break;

    case 'd':
      break;

    case 'l':
      DEBUG_SERIAL.println(" ");
      dxl_debug_menu_shwo_ctrltbl();
      break;

    default:
      exit_menu = true;
      break;
  }


  if(exit_menu == false)
  {
    dxl_debug_menu_show_cmdline();
  }

  return exit_menu;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_debug_menu_shwo_ctrltbl
     WORK    :
---------------------------------------------------------------------------*/
bool dxl_debug_menu_shwo_ctrltbl()
{
  uint32_t addr;


  addr = (uint32_t)&p_dxl_mem->Model_Number - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Model_Number    \t "); DEBUG_SERIAL.print  (p_dxl_mem->Model_Number);
  DEBUG_SERIAL.print("\t 0x");              DEBUG_SERIAL.println(p_dxl_mem->Model_Number,HEX);

  addr = (uint32_t)&p_dxl_mem->Firmware_Version - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Firmware_Version\t "); DEBUG_SERIAL.print  (p_dxl_mem->Firmware_Version);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Firmware_Version,HEX);

  addr = (uint32_t)&p_dxl_mem->ID - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t ID              \t "); DEBUG_SERIAL.print  (p_dxl_mem->ID);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->ID,HEX);

  addr = (uint32_t)&p_dxl_mem->Baud - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Baud            \t "); DEBUG_SERIAL.print  (p_dxl_mem->Baud);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Baud,HEX);

  addr = (uint32_t)&p_dxl_mem->Return_Delay_Time - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Return_Delay_Time \t "); DEBUG_SERIAL.print  (p_dxl_mem->Return_Delay_Time);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Return_Delay_Time,HEX);

  addr = (uint32_t)&p_dxl_mem->Status_Return_Level - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Status_Return_Level \t "); DEBUG_SERIAL.print  (p_dxl_mem->Status_Return_Level);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Status_Return_Level,HEX);

  addr = (uint32_t)&p_dxl_mem->Roll_Offset - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Roll_Offset     \t "); DEBUG_SERIAL.print  (p_dxl_mem->Roll_Offset);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Roll_Offset,HEX);

  addr = (uint32_t)&p_dxl_mem->Pitch_Offset - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Pitch_Offset    \t "); DEBUG_SERIAL.print  (p_dxl_mem->Pitch_Offset);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Pitch_Offset,HEX);

  addr = (uint32_t)&p_dxl_mem->Yaw_Offset - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Yaw_Offset      \t "); DEBUG_SERIAL.print  (p_dxl_mem->Yaw_Offset);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Yaw_Offset,HEX);

  addr = (uint32_t)&p_dxl_mem->Dynamixel_Power - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Dynamixel_Power \t "); DEBUG_SERIAL.print  (p_dxl_mem->Dynamixel_Power);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Dynamixel_Power,HEX);

  addr = (uint32_t)&p_dxl_mem->LED - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t LED             \t "); DEBUG_SERIAL.print  (p_dxl_mem->LED);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->LED,HEX);

  addr = (uint32_t)&p_dxl_mem->LED_RGB - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t LED_RGB         \t "); DEBUG_SERIAL.print  (p_dxl_mem->LED_RGB);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->LED_RGB,HEX);

  addr = (uint32_t)&p_dxl_mem->Buzzer - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Buzzer          \t "); DEBUG_SERIAL.print  (p_dxl_mem->Buzzer);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Buzzer,HEX);

  addr = (uint32_t)&p_dxl_mem->Button - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Button          \t "); DEBUG_SERIAL.print  (p_dxl_mem->Button);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Button,HEX);

  addr = (uint32_t)&p_dxl_mem->Voltage - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Voltage         \t "); DEBUG_SERIAL.print  (p_dxl_mem->Voltage);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Voltage,HEX);

  addr = (uint32_t)&p_dxl_mem->Gyro_X - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Gyro_X          \t "); DEBUG_SERIAL.print  (p_dxl_mem->Gyro_X);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Gyro_X,HEX);

  addr = (uint32_t)&p_dxl_mem->Gyro_Y - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Gyro_Y          \t "); DEBUG_SERIAL.print  (p_dxl_mem->Gyro_Y);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Gyro_Y,HEX);

  addr = (uint32_t)&p_dxl_mem->Gyro_Z - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Gyro_Z          \t "); DEBUG_SERIAL.print  (p_dxl_mem->Gyro_Z);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Gyro_Z,HEX);

  addr = (uint32_t)&p_dxl_mem->Acc_X - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Acc_X           \t "); DEBUG_SERIAL.print  (p_dxl_mem->Acc_X);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Acc_X,HEX);

  addr = (uint32_t)&p_dxl_mem->Acc_Y - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Acc_Y           \t "); DEBUG_SERIAL.print  (p_dxl_mem->Acc_Y);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Acc_Y,HEX);

  addr = (uint32_t)&p_dxl_mem->Acc_Z - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Acc_Z           \t "); DEBUG_SERIAL.print  (p_dxl_mem->Acc_Z);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Acc_Z,HEX);

  addr = (uint32_t)&p_dxl_mem->Roll - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Roll            \t "); DEBUG_SERIAL.print  (p_dxl_mem->Roll);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Roll,HEX);

  addr = (uint32_t)&p_dxl_mem->Pitch - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Pitch           \t "); DEBUG_SERIAL.print  (p_dxl_mem->Pitch);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Pitch,HEX);

  addr = (uint32_t)&p_dxl_mem->Yaw - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t Yaw             \t "); DEBUG_SERIAL.print  (p_dxl_mem->Yaw);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->Yaw,HEX);

  addr = (uint32_t)&p_dxl_mem->IMU_Control - (uint32_t)p_dxl_mem;
  DEBUG_SERIAL.print(addr);
  DEBUG_SERIAL.print("\t IMU_Control     \t "); DEBUG_SERIAL.print  (p_dxl_mem->IMU_Control);
  DEBUG_SERIAL.print("\t 0x");                  DEBUG_SERIAL.println(p_dxl_mem->IMU_Control,HEX);

}
