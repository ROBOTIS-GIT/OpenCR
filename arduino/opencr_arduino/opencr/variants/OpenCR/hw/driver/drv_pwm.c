/*
 *  drv_pwm.c
 *
 *  Created on: 2016. 7. 14.
 *      Author: Baram, PBHP
 */

#include "drv_pwm.h"
#include "variant.h"


#define PWM_PERIOD_VALUE  (0xFFFF-1)

TIM_HandleTypeDef         hTIM1;
TIM_HandleTypeDef         hTIM2;
TIM_HandleTypeDef         hTIM3;
TIM_HandleTypeDef         hTIM9;
TIM_HandleTypeDef         hTIM11;
TIM_HandleTypeDef         hTIM12;

TIM_OC_InitTypeDef        hOC1;
TIM_OC_InitTypeDef        hOC2;
TIM_OC_InitTypeDef        hOC3;
TIM_OC_InitTypeDef        hOC9;
TIM_OC_InitTypeDef        hOC11;
TIM_OC_InitTypeDef        hOC12;




uint32_t pwm_freq[PINS_COUNT];
bool     pwm_init[PINS_COUNT];


int drv_pwm_init()
{
  uint32_t i;


  for( i=0; i<PINS_COUNT; i++ )
  {
    pwm_freq[i] = 50000; // 50Khz
    pwm_init[i] = false;
  }
  return 0;
}


static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (int64_t)(x - in_min) * (int64_t)(out_max - out_min) / (in_max - in_min) + out_min;
}

void drv_pwm_set_freq(uint32_t ulPin, uint32_t freq_data)
{
  freq_data = constrain(freq_data, 1, 1000000);
  pwm_freq[ulPin] = freq_data;
}


uint32_t drv_pwm_get_freq(uint32_t ulPin)
{
  return pwm_freq[ulPin];
}

uint8_t drv_pwm_get_init(uint32_t ulPin)
{
  return pwm_init[ulPin];
}

void drv_pwm_release(uint32_t ulPin)
{
  pwm_init[ulPin] = false;
}


void drv_pwm_setup(uint32_t ulPin)
{
  TIM_HandleTypeDef  *pTIM;
  TIM_OC_InitTypeDef *pOC;
  uint32_t tim_ch;
  uint32_t uwPeriodValue;
  uint32_t uwPrescalerValue = 1;
  uint32_t tim_clk;
  bool is_timer_16bit = true;
  

  if( ulPin >= PINS_COUNT )     return;
  if( pwm_init[ulPin] == true ) return;

  pTIM   = g_Pin2PortMapArray[ulPin].TIMx;
  tim_ch = g_Pin2PortMapArray[ulPin].timerChannel;
  tim_clk = SystemCoreClock;

  if( pTIM == &hTIM3 )
  {
    tim_clk /= 2;
    pOC = &hOC3;
    pTIM->Instance = TIM3;
  }
  else if( pTIM == &hTIM1 )
  {
    pOC = &hOC1;
    pTIM->Instance = TIM1;
  }
  else if( pTIM == &hTIM2 )
  {
    is_timer_16bit = false;
    tim_clk /= 2;
    pOC = &hOC2;
    pTIM->Instance = TIM2;
  }
  else if( pTIM == &hTIM9 )
  {
    pOC = &hOC9;
    pTIM->Instance = TIM9;
  }
  else if( pTIM == &hTIM11 )
  {
    pOC = &hOC11;
    pTIM->Instance = TIM11;
  }
  else if( pTIM == &hTIM12 )
  {
    tim_clk /= 2;
    pOC = &hOC12;
    pTIM->Instance = TIM12;
  }
  else
  {
    return;
  }

  uwPeriodValue = (uint32_t) (tim_clk / pwm_freq[ulPin]);

  if(is_timer_16bit == true)
  {
    uwPrescalerValue = (uwPeriodValue/0xFFFF) + 1;
    uwPeriodValue /= uwPrescalerValue;
  }

  pTIM->Init.Prescaler         = uwPrescalerValue - 1;
  pTIM->Init.Period            = uwPeriodValue - 1;
  pTIM->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  pTIM->Init.CounterMode       = TIM_COUNTERMODE_UP;
  pTIM->Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(pTIM);

  memset(pOC, 0, sizeof(TIM_OC_InitTypeDef));

  pOC->OCMode       = TIM_OCMODE_PWM1;
  pOC->OCPolarity   = TIM_OCPOLARITY_HIGH;
  pOC->OCFastMode   = TIM_OCFAST_DISABLE;
  pOC->OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  pOC->OCNIdleState = TIM_OCNIDLESTATE_RESET;
  pOC->OCIdleState  = TIM_OCIDLESTATE_RESET;

  pOC->Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch);
  HAL_TIM_PWM_Start(pTIM, tim_ch);
  pwm_init[ulPin] = true;
}


void drv_pwm_set_duty(uint32_t ulPin, uint32_t res, uint32_t ulDuty )
{
  TIM_HandleTypeDef  *pTIM;
  uint32_t tim_ch;
  uint32_t pulse;

  if( ulPin >= PINS_COUNT )     return;
  if( pwm_init[ulPin] == false ) return;

  pTIM   = g_Pin2PortMapArray[ulPin].TIMx;
  tim_ch = g_Pin2PortMapArray[ulPin].timerChannel;


  ulDuty = constrain(ulDuty, 0, (1<<res)-1);
  pulse = map( ulDuty, 0, (1<<res)-1, 0, pTIM->Init.Period+1 );

  switch (tim_ch)
  {
    case TIM_CHANNEL_1:
      pTIM->Instance->CCR1 = pulse;
      break;

    case TIM_CHANNEL_2:
      pTIM->Instance->CCR2 = pulse;
      break;

    case TIM_CHANNEL_3:
      pTIM->Instance->CCR3 = pulse;
      break;

    case TIM_CHANNEL_4:
      pTIM->Instance->CCR4 = pulse;
      break;

    default:
      break; 
  }  
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;


  if( htim->Instance == TIM3 )
  {
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    GPIO_InitStruct.Pin       = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM1 )
  {
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM2 )
  {
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM9 )
  {
    __HAL_RCC_TIM9_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    GPIO_InitStruct.Pin       = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM11 )
  {
    __HAL_RCC_TIM11_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM12 )
  {
    __HAL_RCC_TIM12_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}