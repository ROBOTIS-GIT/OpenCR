#include "lds.h"



#define LDS_PACKET_FIND_HEADER      0
#define LDS_PACKET_PARSING          1






void ldsInit(lds_scan_t *p_scan)
{
  uint32_t i;


  p_scan->angle_min = 0.0;
  p_scan->angle_max = 2.0*M_PI;
  p_scan->angle_increment = (2.0*M_PI/360.0);
  p_scan->range_min = 0.12;
  p_scan->range_max = 3.5;

  p_scan->state = LDS_PACKET_FIND_HEADER;
  p_scan->index = 0;
  p_scan->packet_index = 0;
  p_scan->motor_speed = 0;
  p_scan->scan_time = 0;

  for (i=0; i<360; i++)
  {
    p_scan->data[i].range = 0;
    p_scan->data[i].intensity = 0;
  }

  p_scan->buffer_index = 0;
  for (i=0; i<42; i++)
  {
    p_scan->buffer[i] = 0;
  }
}

bool ldsUpdate(lds_scan_t *p_scan, uint8_t data_in)
{
  static uint32_t pre_time = 0;
  static uint32_t pre_scan_time = 0;
  bool ret = false;

  // time out : 1000ms
  if (millis()-pre_time >= 1000)
  {
    pre_time = millis();
    p_scan->state = LDS_PACKET_FIND_HEADER;
    p_scan->index = 0;
    p_scan->buffer_index = 0;
    p_scan->packet_index = 0;
    p_scan->buffer[0] = 0;
    p_scan->buffer[1] = 0;
  }

  switch(p_scan->state)
  {
    case LDS_PACKET_FIND_HEADER:    
      if (p_scan->buffer_index >= 1)
      {
        p_scan->buffer[1] = data_in;

        if(    p_scan->buffer[0] == 0xFA
            && p_scan->buffer[1] == (0xA0 + p_scan->packet_index))
        {
          p_scan->index = p_scan->packet_index * 6;
          p_scan->buffer_index++;
          p_scan->state = LDS_PACKET_PARSING;
          if (p_scan->buffer[1] == 0xA0)
          {
            p_scan->motor_speed_sum = 0;
            p_scan->scan_time = millis() - pre_scan_time;
            pre_scan_time = millis();
          }
        }
        else
        {
          p_scan->buffer[0] = p_scan->buffer[1];
          p_scan->buffer[1] = 0;
        }        
      }
      else
      {
        p_scan->buffer[p_scan->buffer_index] = data_in;
        p_scan->buffer_index++;
      }
      break;

    case LDS_PACKET_PARSING:
      p_scan->buffer[p_scan->buffer_index++] = data_in;

      if (p_scan->buffer_index >= 42)
      {
        p_scan->motor_speed_sum += (p_scan->buffer[3]<<8) | p_scan->buffer[2];

        for (uint16_t i=0; i<6; i++)
        {
          uint16_t index;
          uint8_t  *p_data;

          index  = 359 - (p_scan->index + i); 
          p_data = &p_scan->buffer[4 + 6*i];
          p_scan->data[index].intensity = (p_data[1]<<8) | p_data[0];
          p_scan->data[index].range     = (p_data[3]<<8) | p_data[2];
          p_scan->data[index].reserved  = (p_data[5]<<8) | p_data[4];
        }

        if (p_scan->packet_index == 60-1)
        {
          p_scan->motor_speed = p_scan->motor_speed_sum / 60; 
          ret = true;
        }
        p_scan->packet_index = (p_scan->packet_index + 1) % 60;
        p_scan->state = LDS_PACKET_FIND_HEADER;
        p_scan->buffer_index = 0;
        p_scan->buffer[0] = 0;
        p_scan->buffer[1] = 0;
      }
      break;    
  }


  return ret;
}

