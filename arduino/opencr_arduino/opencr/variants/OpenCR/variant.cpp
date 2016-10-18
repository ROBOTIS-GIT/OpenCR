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

    {GPIOG, GPIO_PIN_10,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 22 BDPIN_LED_USER_1
    {GPIOE, GPIO_PIN_4,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 23 BDPIN_LED_USER_2
    {GPIOE, GPIO_PIN_5,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 24 BDPIN_LED_USER_3
    {GPIOG, GPIO_PIN_12,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 25 BDPIN_LED_USER_4
    {GPIOG, GPIO_PIN_11,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 26 BDPIN_DIP_SW_1
    {GPIOE, GPIO_PIN_6,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 27 BDPIN_DIP_SW_2
    {GPIOA, GPIO_PIN_4,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 28 BDPIN_SPI_CS_IMU
    {GPIOC, GPIO_PIN_0,   &hADC3,   ADC_CHANNEL_10, NULL   ,   NO_PWM       , NO_EXTI },  // 29 BDPIN_BAT_PWR_ADC
    {GPIOC, GPIO_PIN_3,   &hADC3,   ADC_CHANNEL_13, NULL   ,   NO_PWM       , NO_EXTI },  // 30 BDPIN_CHG_PWR_ADC
    {GPIOF, GPIO_PIN_14,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 31 BDPIN_BUZZER
    {GPIOF, GPIO_PIN_15,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 32 BDPIN_DXL_PWR_EN
    {GPIOG, GPIO_PIN_14,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 33 BDPIN_LED_BATT_CHARGE
    {GPIOC, GPIO_PIN_12,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 34 BDPIN_PUSH_SW_1
    {GPIOG, GPIO_PIN_3,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 35 BDPIN_PUSH_SW_2
    {GPIOG, GPIO_PIN_9,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 36 BDPIN_LED_STATUS
    {GPIOF, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 37
    {GPIOF, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 38
    {GPIOF, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 39

    {GPIOB, GPIO_PIN_0,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 40 OLLO_P1_SIG1
    {GPIOC, GPIO_PIN_8,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 41 OLLO_P1_SIG2
    {GPIOA, GPIO_PIN_7,   &hADC1,   ADC_CHANNEL_7 , NULL   ,   NO_PWM       , 5       },  // 42 OLLO_P1_ADC           EXTI_5
    {GPIOC, GPIO_PIN_5,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 43 OLLO_P2_SIG1
    {GPIOB, GPIO_PIN_1,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 44 OLLO_P2_SIG2
    {GPIOC, GPIO_PIN_4,   &hADC1,   ADC_CHANNEL_14, NULL   ,   NO_PWM       , 6       },  // 45 OLLO_P2_ADC           EXTI_6
    {GPIOF, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 46
    {GPIOF, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 47
    {GPIOF, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 48
    {GPIOF, GPIO_PIN_7,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 49

    {GPIOB, GPIO_PIN_10,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 50 BDPIN_GPIO_1
    {GPIOB, GPIO_PIN_11,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 51 BDPIN_GPIO_2
    {GPIOC, GPIO_PIN_13,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 52 BDPIN_GPIO_3
    {GPIOD, GPIO_PIN_2,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 53 BDPIN_GPIO_4
    {GPIOE, GPIO_PIN_3,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 54 BDPIN_GPIO_5
    {GPIOG, GPIO_PIN_2,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 55 BDPIN_GPIO_6
    {GPIOE, GPIO_PIN_10,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 56 BDPIN_GPIO_7
    {GPIOE, GPIO_PIN_11,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 57 BDPIN_GPIO_8
    {GPIOE, GPIO_PIN_12,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 58 BDPIN_GPIO_9
    {GPIOE, GPIO_PIN_13,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 59 BDPIN_GPIO_10
    {GPIOE, GPIO_PIN_14,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 60 BDPIN_GPIO_11
    {GPIOE, GPIO_PIN_15,  NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 61 BDPIN_GPIO_12
    {GPIOF, GPIO_PIN_0,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 62 BDPIN_GPIO_13
    {GPIOF, GPIO_PIN_1,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 63 BDPIN_GPIO_14
    {GPIOF, GPIO_PIN_2,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 64 BDPIN_GPIO_15
    {GPIOF, GPIO_PIN_3,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 65 BDPIN_GPIO_16
    {GPIOF, GPIO_PIN_4,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 66 BDPIN_GPIO_17
    {GPIOF, GPIO_PIN_5,   NULL,     NO_ADC        , NULL   ,   NO_PWM       , NO_EXTI },  // 67 BDPIN_GPIO_18


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

void serialEvent3() __attribute__((weak));
void serialEvent3() { }

UARTClass Serial1(&huart1, USART6_IRQn, 0, USART6);
UARTClass Serial2(&huart2, USART2_IRQn, 1, USART2);
UARTClass Serial3(&huart3, USART3_IRQn, 2, USART3);

void Tx1_Handler(void){ Serial1.TxHandler(); }
void Rx1_Handler(void){ Serial1.RxHandler(); }
void Tx2_Handler(void){ Serial2.TxHandler(); }
void Rx2_Handler(void){ Serial2.RxHandler(); }
void Tx3_Handler(void){ Serial3.TxHandler(); }
void Rx3_Handler(void){ Serial3.RxHandler(); }

void Err1_Handler(void){ Serial1.ErrHandler(); }
void Err2_Handler(void){ Serial2.ErrHandler(); }
void Err3_Handler(void){ Serial3.ErrHandler(); }



void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
  if (Serial3.available()) serialEvent3();
}

USBSerial Serial;

