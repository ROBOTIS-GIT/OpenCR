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
 /*
 *  Modified on: 2016. 7.12.
 *       Author: Baram, PBHP
 */
#include "Arduino.h"




#ifdef __cplusplus
extern "C" {
#endif




extern const Pin2PortMapArray g_Pin2PortMapArray[]=
{
    {GPIOC, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 0  UART6_RX
    {GPIOC, GPIO_PIN_6,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 1  UART6_TX
    {GPIOG, GPIO_PIN_6,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , 0       },  // 2                         EXTI_0
    {GPIOB, GPIO_PIN_4,   NULL,     NO_ADC        , &hTIM3 ,   TIM_CHANNEL_1, 1       },  // 3  TIM3_CH1               EXTI_1
    {GPIOG, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , 2       },  // 4                         EXTI_2
    {GPIOA, GPIO_PIN_8,   NULL,     NO_ADC        , &hTIM1 ,   TIM_CHANNEL_1, NO_EXTI },  // 5  TIM1_CH1
    {GPIOA, GPIO_PIN_2,   NULL,     NO_ADC        , &hTIM2 ,   TIM_CHANNEL_3, NO_EXTI },  // 6  TIM2_CH3
    {GPIOC, GPIO_PIN_1,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , 3       },  // 7                         EXTI_3
    {GPIOC, GPIO_PIN_2,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , 4       },  // 8                         EXTI_4
    {GPIOA, GPIO_PIN_3,   NULL,     NO_ADC        , &hTIM9 ,   TIM_CHANNEL_2, NO_EXTI },  // 9  TIM9_CH2
    {GPIOB, GPIO_PIN_9,   NULL,     NO_ADC        , &hTIM11,   TIM_CHANNEL_1, NO_EXTI },  // 10 TIM11_CH1   SPI2_NSS
    {GPIOB, GPIO_PIN_15,  NULL,     NO_ADC        , &hTIM12,   TIM_CHANNEL_2, NO_EXTI },  // 11 TIM12_CH2   SPI2_MOSI
    {GPIOB, GPIO_PIN_14,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 12             SPI2_MISO
    {GPIOA, GPIO_PIN_9,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 13 LED         SPI2_SCK
    {GPIOB, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 14             I2C1_SDA
    {GPIOB, GPIO_PIN_8,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 15             I2C1_SCL

    {GPIOA, GPIO_PIN_0,   &hADC3,   ADC_CHANNEL_0 , NULL   ,   NO_PWM       , NO_EXTI },  // 16 A0
    {GPIOF, GPIO_PIN_10,  &hADC3,   ADC_CHANNEL_8 , NULL   ,   NO_PWM       , NO_EXTI },  // 17 A1
    {GPIOF, GPIO_PIN_9,   &hADC3,   ADC_CHANNEL_7 , NULL   ,   NO_PWM       , NO_EXTI },  // 18 A2
    {GPIOF, GPIO_PIN_8,   &hADC3,   ADC_CHANNEL_6 , NULL   ,   NO_PWM       , NO_EXTI },  // 19 A3
    {GPIOF, GPIO_PIN_7,   &hADC3,   ADC_CHANNEL_5 , NULL   ,   NO_PWM       , NO_EXTI },  // 20 A4
    {GPIOF, GPIO_PIN_6,   &hADC3,   ADC_CHANNEL_4 , NULL   ,   NO_PWM       , NO_EXTI },  // 21 A5

    {GPIOB, GPIO_PIN_10,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 22 TEST_PIN_1
    {GPIOB, GPIO_PIN_11,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 23 TEST_PIN_2
    {GPIOC, GPIO_PIN_13,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 24 TEST_PIN_3
    {GPIOD, GPIO_PIN_2,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 25 TEST_PIN_4
    {GPIOE, GPIO_PIN_3,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 26 TEST_PIN_5
    {GPIOG, GPIO_PIN_2,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 27 TEST_PIN_6
    {GPIOA, GPIO_PIN_4,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 28 MPU CS
    {GPIOC, GPIO_PIN_0,   &hADC3,   ADC_CHANNEL_10, NULL   ,   NO_PWM       , NO_EXTI },  // 29 BAT
    {GPIOC, GPIO_PIN_15,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 30
    {NULL , 0          ,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI }
};


#ifdef __cplusplus
}
#endif




/* ----------------------------------------------------------------------------
 *     USART objects
 * ----------------------------------------------------------------------------*/
void serialEvent() __attribute__((weak));
void serialEvent() { }

void serialEvent1() __attribute__((weak));
void serialEvent1() { }

void serialEvent2() __attribute__((weak));
void serialEvent2() { }


UARTClass Serial1(&huart1, USART6_IRQn, 0, USART6);
UARTClass Serial2(&huart2, USART2_IRQn, 1, USART2);

void Tx1_Handler(void){ Serial1.TxHandler(); }
void Rx1_Handler(void){ Serial1.RxHandler(); }
void Tx2_Handler(void){ Serial2.TxHandler(); }
void Rx2_Handler(void){ Serial2.RxHandler(); }




void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
}

USBSerial Serial;

