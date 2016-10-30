/*
   Copyright (c) 2011 Arduino.  All right reserved.
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
   See the GNU Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include "Arduino.h"
#include "variant.h"

uint8_t analog_reference = DEFAULT;


static int _readResolution = 10;
static int _writeResolution = 8;
int readResolBackup = -1;
int writeResolBackup = -1;



void analogReadResolution(int res) {
	_readResolution = res;
}


void analogWriteResolution(int res) {
	_writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

void analogReference(uint8_t mode){
	analog_reference = mode;
}

uint32_t analogRead( uint32_t ulPin ){ 
 
  ADC_ChannelConfTypeDef sConfig;
  ADC_HandleTypeDef      *hADCx;
	uint32_t ulValue = 0;
  uint32_t ulChannel;
  uint32_t adc_pin;


  adc_pin = analogPinToChannel(ulPin);

  ulChannel = g_Pin2PortMapArray[adc_pin].adc_channel;
  
  if(ulChannel == NO_ADC)
      return -1;

  hADCx = g_Pin2PortMapArray[adc_pin].ADCx;

  sConfig.Channel      = ulChannel;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset       = 0;
  HAL_ADC_ConfigChannel(hADCx, &sConfig);
  
  HAL_ADC_Start(hADCx);
  HAL_ADC_PollForConversion(hADCx, 10);
  ulValue = HAL_ADC_GetValue(hADCx);

  ulValue = mapResolution(ulValue, 12, _readResolution);
 
  return ulValue;
}


void analogWrite( uint32_t ulPin, uint32_t ulValue ){

  if( drv_pwm_get_init(ulPin) == false )
  {
    drv_pwm_setup(ulPin);
  }
  
  drv_pwm_set_duty(ulPin, _writeResolution, ulValue);
}
