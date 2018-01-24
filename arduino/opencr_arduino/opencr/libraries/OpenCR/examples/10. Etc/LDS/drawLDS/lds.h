#ifndef _LDS_H_
#define _LDS_H_


#include <Arduino.h>


typedef struct
{
  uint16_t range;
  uint16_t intensity;
  uint16_t reserved;
} lds_packet_t;


typedef struct
{
  uint8_t state;
  uint8_t buffer_index;
  uint8_t buffer[42];
  uint16_t index;
  uint16_t packet_index;
  uint32_t motor_speed;
  uint32_t motor_speed_sum;
  uint32_t scan_time;
  
  float angle_min;
  float angle_max;
  float angle_increment;
  float range_min;
  float range_max;

  lds_packet_t data[360];
} lds_scan_t;




void ldsInit(lds_scan_t *p_scan);
bool ldsUpdate(lds_scan_t *p_scan, uint8_t data_in);


#endif
