/*
 *  dxl_hw_op3.h
 *
 *  dynamixel hardware op3
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#ifndef DXL_HW_OP3_H
#define DXL_HW_OP3_H


#include "dxl_def.h"


#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#define PIN_LED_R         50
#define PIN_LED_G         51
#define PIN_LED_B         52

#define PIN_LED_1         53
#define PIN_LED_2         54
#define PIN_LED_3         55

#define PIN_BUTTON_S1     56
#define PIN_BUTTON_S2     57
#define PIN_BUTTON_S3     58
#define PIN_BUTTON_S4     59





void dxl_hw_op3_init(void);
void dxl_hw_op3_update(void);

// button
uint8_t dxl_hw_op3_button_read(uint8_t pin_num);

// led
void dxl_hw_op3_led_set(uint8_t pin_num, uint8_t value);
void dxl_hw_op3_led_pwm(uint8_t pin_num, uint8_t value);

// voltage
uint8_t dxl_hw_op3_voltage_read(void);

// acc
int16_t dxl_hw_op3_gyro_get_x(void);
int16_t dxl_hw_op3_gyro_get_y(void);
int16_t dxl_hw_op3_gyro_get_z(void);

int16_t dxl_hw_op3_acc_get_x(void);
int16_t dxl_hw_op3_acc_get_y(void);
int16_t dxl_hw_op3_acc_get_z(void);

int16_t dxl_hw_op3_get_rpy(uint8_t rpy);
void    dxl_hw_op3_start_cali(uint8_t index);
int16_t dxl_hw_op3_get_cali(uint8_t index);
void    dxl_hw_op3_clear_cali(uint8_t index);


void  dxl_hw_op3_set_offset(uint8_t index, float offset_data);
float dxl_hw_op3_get_offset(uint8_t index);

void dxl_hw_op3_start_gyro_cali(void);
bool dxl_hw_op3_get_gyro_cali_done(void);


#endif
