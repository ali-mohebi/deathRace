/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */

#include "LiquidCrystal.h"




#define Blink 1
#define Start 2
#define Level_1 3
#define Level_2 4
#define Level_3 5
#define Level_4 6
#define Level_5 7
#define Level_6 8
#define Level_7 9
#define Level_8 10
#define Level_final 11
#define Fail 12


uint8_t LDR_Value = 0;
uint8_t POTENTIOMETER_Value = 50;

uint8_t Location_Macine;

uint8_t state = 0;


/*
#define ARRAY_Number 10
#define TIME_Level 15000
#define Time_Fall_Level_1 1500
#define Time_Fall_Level_2 1358
#define Time_Fall_Level_3 1216
#define Time_Fall_Level_4 1074
#define Time_Fall_Level_5 932
#define Time_Fall_Level_6 790
#define Time_Fall_Level_7 648
#define Time_Fall_Level_8 506
#define Time_Fall_Level_final 506


uint32_t Count_Level = 0;
uint32_t Time_Fall = 0;
uint8_t Row_Fall = -1;
uint8_t Column_Fall = 0;
uint8_t SW_Disply = 0;
*/

//enum{Blink, Start, Level_1, Level_2, Level_3, Level_4, Level_5, Level_6, Level_7, Level_8, Level_final, Fail} state;



//uint32_t Rand_Time_Array[ARRAY_Number]={0};
//uint32_t Rand_Location_Array[ARRAY_Number]={0};

//uint8_t Dike_Number = 0;




//char str[5];















/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

	
	if(state == Blink)
	{
		state = Start;
	}
	

	
		
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	
	
	
	
	POTENTIOMETER_Value = HAL_ADC_GetValue(&hadc1);
	POTENTIOMETER_Value = (POTENTIOMETER_Value * 100) / 63;
	
	
	if(POTENTIOMETER_Value <= 10) Location_Macine = 1;
	else if((POTENTIOMETER_Value > 10) && (POTENTIOMETER_Value <= 15)) Location_Macine = 2;
	else if((POTENTIOMETER_Value > 15) && (POTENTIOMETER_Value <= 20)) Location_Macine = 3;
	else if((POTENTIOMETER_Value > 20) && (POTENTIOMETER_Value <= 25)) Location_Macine = 4;
	else if((POTENTIOMETER_Value > 25) && (POTENTIOMETER_Value <= 30)) Location_Macine = 5;
	else if((POTENTIOMETER_Value > 30) && (POTENTIOMETER_Value <= 35)) Location_Macine = 6;
	else if((POTENTIOMETER_Value > 35) && (POTENTIOMETER_Value <= 40)) Location_Macine = 7;
	else if((POTENTIOMETER_Value > 40) && (POTENTIOMETER_Value <= 45)) Location_Macine = 8;
	else if((POTENTIOMETER_Value > 45) && (POTENTIOMETER_Value <= 50)) Location_Macine = 9;
	else if((POTENTIOMETER_Value > 50) && (POTENTIOMETER_Value <= 55)) Location_Macine = 10;
	else if((POTENTIOMETER_Value > 55) && (POTENTIOMETER_Value <= 60)) Location_Macine = 11;
	else if((POTENTIOMETER_Value > 60) && (POTENTIOMETER_Value <= 65)) Location_Macine = 12;
	else if((POTENTIOMETER_Value > 65) && (POTENTIOMETER_Value <= 70)) Location_Macine = 13;
	else if((POTENTIOMETER_Value > 70) && (POTENTIOMETER_Value <= 75)) Location_Macine = 14;
	else if((POTENTIOMETER_Value > 75) && (POTENTIOMETER_Value <= 80)) Location_Macine = 15;
	else if((POTENTIOMETER_Value > 80) && (POTENTIOMETER_Value <= 85)) Location_Macine = 16;
	else if(POTENTIOMETER_Value > 85) Location_Macine = 17;
	
	
	HAL_ADC_Start_IT(&hadc1);
	
	

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
	
	
	/*
	
	Count_Level++;
	Time_Fall++;
	
	
	switch(state)
	{
		case Level_1:
			
		
			
			if(Time_Fall == Time_Fall_Level_1)
			{
				Row_Fall++;
				if(Row_Fall == 4) Row_Fall = 0;
				Time_Fall = 0;
			}
			
			
			
			
			
			if(Dike_Number != 3)
			{
				if(Count_Level == Rand_Time_Array[0])
				{
					Dike_Number++;	
					
					SW_Disply = 1;
					
					Column_Fall = Rand_Location_Array[0];
				}
				else if(Count_Level == Rand_Time_Array[1])
				{
					Dike_Number++;
				}
				else if(Count_Level == Rand_Time_Array[2])
				{
					Dike_Number++;
				}
			}
			break;
			
			
			
			
			
		case Level_2:
			//state = Level_3;
			break;
		case Level_3:
			//state = Level_4;
			break;
		case Level_4:
			//state = Level_5;
			break;
		case Level_5:
			//state = Level_6;
			break;
		case Level_6:
			//state = Level_7;
			break;
		case Level_7:
			//state = Level_8;
			break;
		case Level_8:
			//state = Level_final;
			break;
	}
	
	
	
	
	
	if(Count_Level == TIME_Level)
	{
		Count_Level = 0;
		Time_Fall = 0;
		Dike_Number = 0;
		
		switch(state)
		{
			case Level_1:
				state = Level_2;
				break;
			case Level_2:
				state = Level_3;
				break;
			case Level_3:
				state = Level_4;
				break;
			case Level_4:
				state = Level_5;
				break;
			case Level_5:
				state = Level_6;
				break;
			case Level_6:
				state = Level_7;
				break;
			case Level_7:
				state = Level_8;
				break;
			case Level_8:
				state = Level_final;
				break;
		}
		
	}
	
	
	
	HAL_TIM_Base_Start_IT(&htim7);
	*/
	
	
	
	

  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles ADC4 interrupt.
*/
void ADC4_IRQHandler(void)
{
  /* USER CODE BEGIN ADC4_IRQn 0 */

  /* USER CODE END ADC4_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc4);
  /* USER CODE BEGIN ADC4_IRQn 1 */
	
	
	
	LDR_Value = HAL_ADC_GetValue(&hadc4);
	LDR_Value = (LDR_Value * 100) / 63;
	
	HAL_ADC_Start_IT(&hadc4);
	
	

  /* USER CODE END ADC4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
