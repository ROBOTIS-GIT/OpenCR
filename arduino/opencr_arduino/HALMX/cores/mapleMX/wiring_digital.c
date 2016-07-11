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

	/*
		in example code this is typically local
	*/
	GPIO_InitTypeDef  GPIO_InitStruct;
	

	/* 
		to set the mode through the HAL libraries we populate the
		structure.  If STM32CubeMX is used to enable the pin
		or peripheral this code becomes redundant and may
		conflict.
		
		an option to overcome this would be to set a flag
		in the startup code that ignores this if the port is
		already in use.
		
		first we need to get the port from the pin2port array
		
	*/

	GPIO_InitStruct.Pin = g_Pin2PortMapArray[ulPin].Pin_abstraction;
	
	/* check the port to see if it is a pin we can modify */
// from sam code -- needs work ---
//	if ( g_Pin2PortMapArray[ulPin].ulPinType == PIO_NOT_A_PIN )
//    {
//        return ;
//    }
	/* 
		Taken from blink led example
		
		there may need to be some additional methods to
		allow for some of the additional hardware modes
		that the STM32 can provide. these can and are set by the HAL init
		code for specific peripherals this is included here to provide
		existing Arduino library setup code in simple bitbang
		examples.
		Using this to setup built in peripherals probably will
		conflict and cause unpredictable behavior
	*/

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

        case OUTPUT:
 			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 			GPIO_InitStruct.Pull = GPIO_NOPULL;
		break ;

        default:
        break ;
    }

	/*
		this speed mapping taken from blink example
		
		There may be some chip specific issues here
		Roger Clark identified that the F1xx HAL does
		not define FAST for speed.
		
		Online examples of Blink are all over the place on
		how this should be set.
		
	*/
  	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  	
  	/*
  		now we can call the actual setup code note that the
  		port reference is an opaque data type which
  		is typically a pointer.  This makes it awkward
  		to start the port clocks locally through a switch statement
  		on the port reference. We could shadow the port reference
  		as an enum, but would require a separate port peripheral
  		definition casts to a switchable int. 
  		
  		Currently port clocks are set as part of the gpio_h includes
  		from CubeMX.  This or an equivalent should be called
  		as part of main() as part of the HAL startup
  		
  		If no gpio is configured in CubeMX the gpio startup
  		will not be called and cause conflict with the clock system
  		
  		It may be necessary to provide some assert or
  		compiler warning setup should this code be used for general use. 
  		
  	*/
  	HAL_GPIO_Init(g_Pin2PortMapArray[ulPin].GPIOx_Port, &GPIO_InitStruct); 


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

	/* can add a section here to see if pin is readable */
	
	
	if(HAL_GPIO_ReadPin(g_Pin2PortMapArray[ulPin].GPIOx_Port,g_Pin2PortMapArray[ulPin].Pin_abstraction) == GPIO_PIN_RESET)
	{
		return LOW; // Set from HIGH to LOW by Vassilis Serasidis
	}

	return HIGH ; // Set from LOW to HIGH by Vassilis Serasidis
}

#ifdef __cplusplus
}
#endif

