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

volatile TIM_OC_InitTypeDef        hOC1;
volatile TIM_OC_InitTypeDef        hOC2;
volatile TIM_OC_InitTypeDef        hOC3;
volatile TIM_OC_InitTypeDef        hOC9;
volatile TIM_OC_InitTypeDef        hOC11;
volatile TIM_OC_InitTypeDef        hOC12;




uint32_t pwm_freq[64];
bool     pwm_init[64];




int drv_pwm_init()
{
  uint32_t i;


  for( i=0; i<64; i++ )
  {
    pwm_freq[i] = 50000; // 50Khz
    pwm_init[i] = false;
  }
  return 0;
}


static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drv_pwm_set_freq(uint32_t ulPin, uint32_t freq_data)
{
  pwm_freq[ulPin] = freq_data;
}


void drv_pwm_get_freq(uint32_t ulPin)
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


  if( pwm_init[ulPin] == true ) return;


  pTIM   = g_Pin2PortMapArray[ulPin].TIMx;
  tim_ch = g_Pin2PortMapArray[ulPin].timerChannel;
  uwPeriodValue = (uint32_t) (((SystemCoreClock/2)  / pwm_freq[ulPin]) - 1);

  if( pTIM == &hTIM3 )
  {
    pOC = &hOC3;
    pTIM->Instance = TIM3;
  }
  else if( pTIM == &hTIM1 )
  {
    uwPeriodValue = (uint32_t) (((SystemCoreClock)  / pwm_freq[ulPin]) - 1);
    pOC = &hOC1;
    pTIM->Instance = TIM1;
  }
  else if( pTIM == &hTIM2 )
  {
    pOC = &hOC2;
    pTIM->Instance = TIM2;
  }
  else if( pTIM == &hTIM9 )
  {
    uwPeriodValue = (uint32_t) (((SystemCoreClock)  / pwm_freq[ulPin]) - 1);
    pOC = &hOC9;
    pTIM->Instance = TIM9;
  }
  else if( pTIM == &hTIM11 )
  {
    uwPeriodValue = (uint32_t) (((SystemCoreClock)  / pwm_freq[ulPin]) - 1);
    pOC = &hOC11;
    pTIM->Instance = TIM11;
  }
  else if( pTIM == &hTIM12 )
  {
    pOC = &hOC12;
    pTIM->Instance = TIM12;
  }
  else
  {
    return;
  }

  pTIM->Init.Prescaler         = 0;
  pTIM->Init.Period            = uwPeriodValue;
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
  TIM_OC_InitTypeDef *pOC;
  uint32_t tim_ch;


  if( pwm_init[ulPin] == false ) return;


  pTIM   = g_Pin2PortMapArray[ulPin].TIMx;
  tim_ch = g_Pin2PortMapArray[ulPin].timerChannel;


  if     ( pTIM->Instance == TIM3 ) pOC = &hOC3;
  else if( pTIM->Instance == TIM1 ) pOC = &hOC1;
  else if( pTIM->Instance == TIM2 ) pOC = &hOC2;
  else if( pTIM->Instance == TIM9 ) pOC = &hOC9;
  else if( pTIM->Instance == TIM11 ) pOC = &hOC11;
  else if( pTIM->Instance == TIM12 ) pOC = &hOC12;
  else
  {
    return;
  }


  ulDuty = constrain(ulDuty, 0, (1<<res)-1);
  pOC->Pulse = map( ulDuty, 0, (1<<res)-1, 0, pTIM->Init.Period+1 );
  HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch);
  HAL_TIM_PWM_Start(pTIM, tim_ch);
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;


  if( htim->Instance == TIM3 )
  {
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    GPIO_InitStruct.Pin       = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM1 )
  {
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM2 )
  {
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM9 )
  {
    __HAL_RCC_TIM9_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    GPIO_InitStruct.Pin       = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM11 )
  {
    __HAL_RCC_TIM11_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM12 )
  {
    __HAL_RCC_TIM12_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

