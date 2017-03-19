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

#ifdef __cplusplus
 extern "C" {
#endif

	/*

		the HAL drivers for STM32 do no access the registers directly
		opaque structures are used to abstract the functions needed

		existing STM examples show this defined locally a static
		'static GPIO_InitTypeDef  GPIO_InitStruct;'

		value which we set here.  This is different than the
		way maple and sam set the pins as they use a global
		array that contains device specific register setting values


		GPIO_InitTypeDef has the following members defined in stm32f4xx_hal_gpio.h:

  uint32_t Pin		Specifies the GPIO pins to be configured.
					typically a bitmapped mask that can be stored in a 16 bit value

  uint32_t Mode		Specifies the operating mode for the selected pins.
                    There are a number of modes that can be defined.
GPIO_MODE_INPUT		        	Input Floating Mode
GPIO_MODE_OUTPUT_PP        		Output Push Pull Mode
GPIO_MODE_OUTPUT_OD        		Output Open Drain Mode
GPIO_MODE_AF_PP            		Alternate Function Push Pull Mode
GPIO_MODE_AF_OD            		Alternate Function Open Drain Mode

GPIO_MODE_ANALOG           		Analog Mode

GPIO_MODE_IT_RISING        		External Interrupt Mode with Rising edge trigger detection
GPIO_MODE_IT_FALLING       		External Interrupt Mode with Falling edge trigger detection
GPIO_MODE_IT_RISING_FALLING		External Interrupt Mode with Rising/Falling edge trigger detection

GPIO_MODE_EVT_RISING       		External Event Mode with Rising edge trigger detection
GPIO_MODE_EVT_FALLING      		External Event Mode with Falling edge trigger detection
GPIO_MODE_EVT_RISING_FALLING	External Event Mode with Rising/Falling edge trigger detection


  uint32_t Pull		Specifies the Pull-up or Pull-Down activation for the selected pins.
GPIO_NOPULL						No Pull-up or Pull-down activation
GPIO_PULLUP						Pull-up activation
GPIO_PULLDOWN					Pull-down activation

  uint32_t Speed	Specifies the speed for the selected pins.
GPIO_SPEED_LOW					Low speed
GPIO_SPEED_MEDIUM				Medium speed
GPIO_SPEED_FAST					Fast speed
GPIO_SPEED_HIGH					High speed

  uint32_t Alternate	Peripheral to be connected to the selected pins.
                        these are defined in stm32f4xx_hal_gpio_ex.h
                        not all calls to HAL_GPIO_Init require this value to
                        be set in the existing example code


*/




extern void pinMode( uint32_t ulPin, uint32_t ulMode )
{
	GPIO_InitTypeDef  GPIO_InitStruct;


	GPIO_InitStruct.Pin = g_Pin2PortMapArray[ulPin].Pin_abstraction;

	drv_pwm_release(ulPin);


	switch ( ulMode )
	{
	  case INPUT:
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    break ;

	  case INPUT_PULLUP:
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			break ;

	  case INPUT_PULLDOWN:
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    break;

	  case OUTPUT:
 			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 			GPIO_InitStruct.Pull = GPIO_NOPULL;
 			break ;

    case OUTPUT_OPEN:
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
 			GPIO_InitStruct.Pull = GPIO_NOPULL;
      break;

	  case INPUT_ANALOG:
	    drv_adc_pin_init(ulPin);
	    break;

	  default:
	    break ;
	}

	if( ulMode != INPUT_ANALOG )
	{
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  HAL_GPIO_Init(g_Pin2PortMapArray[ulPin].GPIOx_Port, &GPIO_InitStruct);
	}
}

extern void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{

  switch(ulVal) {

    case HIGH:
			/*
					AVR allows for the writing of inputs to set the pull up state
					we may want to do this here as well to maintain compatibility.
			*/
      HAL_GPIO_WritePin(g_Pin2PortMapArray[ulPin].GPIOx_Port,g_Pin2PortMapArray[ulPin].Pin_abstraction,GPIO_PIN_SET);
			break;

    case LOW:
      /* simply reset the pin */
      HAL_GPIO_WritePin(g_Pin2PortMapArray[ulPin].GPIOx_Port,g_Pin2PortMapArray[ulPin].Pin_abstraction,GPIO_PIN_RESET);
			break;

    default:
      /* should do an assert here to handle error conditions */
			break;
  }
}

extern int digitalRead( uint32_t ulPin )
{
	if(HAL_GPIO_ReadPin(g_Pin2PortMapArray[ulPin].GPIOx_Port,g_Pin2PortMapArray[ulPin].Pin_abstraction) == GPIO_PIN_RESET)
	{
		return LOW; // Set from HIGH to LOW by Vassilis Serasidis
	}

	return HIGH ; // Set from LOW to HIGH by Vassilis Serasidis
}

#ifdef __cplusplus
}
#endif
