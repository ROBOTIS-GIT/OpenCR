/****************************************************************************
 *
 * wiring_analog.c for Arduino STM32 + HAL + CubeMX (HALMX).
 *
 * Copyright (c) 2016 by Vassilis Serasidis <info@serasidis.gr>
 * Home: http://www.serasidis.gr
 * email: avrsite@yahoo.gr
 * 
 * Arduino_STM32 forum: http://www.stm32duino.com
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 ****************************************************************************/

#include "Arduino.h"
//#include <chip.h>
//#include "wiring_private.h"
#include "variant.h"
//#include "pins_arduino.h"

uint8_t analog_reference = DEFAULT;
TIM_HandleTypeDef* _htimX;

static int _readResolution = 10;
static int _writeResolution = 8;
int readResolBackup = -1;
int writeResolBackup = -1;
uint32_t ulValBackup = 0;
uint32_t enabledPWMpins[MAX_PWM_PIN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t _ulPin = 0xffffffff;

/*********************************************************
 *
 *
 *********************************************************/
void analogReadResolution(int res) {
	_readResolution = res;
}

/*********************************************************
 *
 *
 *********************************************************/
void analogWriteResolution(int res) {
	_writeResolution = res;
}

/*********************************************************
 * Converts the <value> from <from> bits to <to> bits.
 * Example: covert the value 1024 from 10 bit to 8 bit
 *          it returns the value 256 .
 *********************************************************/
static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

/*********************************************************
 *
 *
 *********************************************************/
void analogReference(uint8_t mode){
	analog_reference = mode;
}

/*********************************************************
 * Reads an analog input value (12-bit) and divide it 
 * according to the _readResolution register (8 - 12 bit)
 *********************************************************/
uint32_t analogRead( uint32_t ulPin ){ 
 
  ADC_ChannelConfTypeDef sConfig;
	uint32_t ulValue = 0;
  uint32_t ulChannel;

  ulChannel = g_Pin2PortMapArray[ulPin].adc_channel;
  
  if(ulChannel == NO_ADC) //If that pin doesn't have ADC interface, then exit.
      return -1;
    
  sConfig.Channel = ulChannel; //Change ADC channel.
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  
  HAL_ADC_Start(&hadc1); //Start the ADC conversion.
  HAL_ADC_PollForConversion(&hadc1, 1000000);
  ulValue = HAL_ADC_GetValue(&hadc1);

  ulValue = mapResolution(ulValue, 12, _readResolution);
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
 
  return ulValue;
}


/*********************************************************
 * Writes the <ulValue> value to the PWM <ulPin> pin. 
 *
 *********************************************************/
void analogWrite( uint32_t ulPin, uint32_t ulValue ){
  /* If previous PWM pin is not the same with the one is used now, check if it has been configured. */
  if(ulPin != _ulPin){ 
    uint8_t i, res = 0;
    for(i=0;i<MAX_PWM_PIN;i++){
      if(enabledPWMpins[i] == ulPin){
        enabledPWMpins[i] = ulPin;
        res = 1;
      }
    }
    if(res == 1)
      MX_TIMx_Init(ulPin);
    _ulPin = ulPin;
  }
  
  TIM_OC_InitTypeDef sConfigOC;
  uint32_t ulTemp = 256;
  uint32_t ulChannel = g_Pin2PortMapArray[ulPin].timerChannel;
  
  /* Check if the current resolusion is not the same with the previous one */
  if(writeResolBackup != _writeResolution){
    ulTemp = mapResolution(ulTemp, _writeResolution, 8); 
    writeResolBackup = _writeResolution;
    _htimX->Init.Period = pow(2, _writeResolution);
    HAL_TIM_PWM_Init(_htimX);
  }
   
   /* Set the new pulse width */
   if(ulValBackup != ulValue){
     ulValBackup = ulValue;
     sConfigOC.OCMode = TIM_OCMODE_PWM1;
		 sConfigOC.Pulse = ulValue;
     HAL_TIM_PWM_ConfigChannel(_htimX, &sConfigOC, ulChannel);
     HAL_TIM_PWM_Start(_htimX, ulChannel);
   }
   
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/*********************************************************
 *  TIMx init function 
 *  Enable the PWM functionality on <ulPin> pin.
 *********************************************************/
void MX_TIMx_Init(uint32_t ulPin)
{
    switch(g_Pin2PortMapArray[ulPin].timerNumber){
    case 1:
      _htimX = &htim1;
      _htimX->Instance = TIM1;
    break;
    case 2:
      _htimX = &htim2;
      _htimX->Instance = TIM2;
    break;
    case 3:
      _htimX = &htim3;
      _htimX->Instance = TIM3;
    break;
    case 4:
      _htimX = &htim4;
      _htimX->Instance = TIM3;
    break;
  }
  
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  _htimX->Init.Prescaler = 0;
  _htimX->Init.CounterMode = TIM_COUNTERMODE_UP;
  _htimX->Init.Period = 255;
  _htimX->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(_htimX);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(_htimX, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  HAL_TIM_PWM_ConfigChannel(_htimX, &sConfigOC, g_Pin2PortMapArray[ulPin].timerChannel);
  
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = g_Pin2PortMapArray[ulPin].Pin_abstraction;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(g_Pin2PortMapArray[ulPin].GPIOx_Port, &GPIO_InitStruct);
  if(_htimX->Instance == TIM3)
    __HAL_AFIO_REMAP_TIM3_PARTIAL();
  
  HAL_TIM_PWM_Init(_htimX);
  HAL_TIM_PWM_Start(_htimX, g_Pin2PortMapArray[ulPin].timerChannel);
}
