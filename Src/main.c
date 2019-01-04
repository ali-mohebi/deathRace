/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */


#include "LiquidCrystal.h"
#include "stdlib.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/*
#define Blink 0
#define Start 1
#define Level_1 2
#define Level_2 3
#define Level_3 4
#define Level_4 5
#define Level_5 6
#define Level_6 7
#define Level_7 8
#define Level_8 9
#define Level_final 10
#define Fail 11
*/

#define TIME_Blink 100
#define TIME_Level 15000
#define TIME_1S 1000
#define Turbo_TIME 5000

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

#define Time_Fall_Level_1 1500
#define Time_Fall_Level_2 1358
#define Time_Fall_Level_3 1216
#define Time_Fall_Level_4 1074
#define Time_Fall_Level_5 932
#define Time_Fall_Level_6 790
#define Time_Fall_Level_7 648
#define Time_Fall_Level_8 506
#define Time_Fall_Level_final 506

#define ARRAY_Number 10


//extern enum{Blink, Start, Level_1, Level_2, Level_3, Level_4, Level_5, Level_6, Level_7, Level_8, Level_final, Fail} state;
//enum{Blink, Start, Level_1, Level_2, Level_3, Level_4, Level_5, Level_6, Level_7, Level_8, Level_final, Fail} state;

extern uint8_t LDR_Value;
extern uint8_t POTENTIOMETER_Value;

extern uint8_t state;

uint32_t counter = 0;
uint32_t Count_Level = 0;
uint32_t Time_Fall = 0;
uint8_t SW_Disply = 0;

uint8_t SW_Blink = 0;

uint8_t SW_Rand = 0;

uint32_t Rand_Time_Array[ARRAY_Number];
uint32_t Rand_Location_Array[ARRAY_Number];

extern uint8_t Location_Macine;

uint8_t CC1 = 1, CC2 = 2, CC3 = 3, CC4 = 4, CC5 = 5;

uint8_t Dike_Number = 0;
uint8_t Dike_Array[10] = {0};
uint8_t Row_Fall[10] = {0};
uint8_t Column_Fall[10] = {0};

uint32_t Count_Time = 0;
uint32_t Second = 0, Minute = 0;
uint8_t Select_Segment = 0;

uint8_t Speed = 0;
uint32_t Count_Turbo = 0;
int8_t Turbo = -1;






char str[5];
//char str;



uint8_t customChar1[8] = {
	0x07,
	0x02,
	0x14,
	0x1F,
	0x13,
	0x1F,
	0x0C,
	0x0C
};
uint8_t customChar2[8] = {
	0x1C,
	0x08,
	0x05,
	0x1F,
	0x19,
	0x1F,
	0x06,
	0x06
};
uint8_t customChar3[8] = {
	0x01,
	0x03,
	0x06,
	0x04,
	0x0C,
	0x08,
	0x18,
	0x10
};
uint8_t customChar4[8] = {
	0x10,
	0x18,
	0x0C,
	0x04,
	0x06,
	0x02,
	0x03,
	0x01
};
uint8_t customChar5[8] = {
	0x00,
	0x0A,
	0x1F,
	0x1F,
	0x1F,
	0x0E,
	0x04,
	0x00
};








/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC4_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM7_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);





/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void Turbo_Charge()
{
	switch(Turbo)
	{
		case 0:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,LDR_Value);  //LED0
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,LDR_Value);  //LED1
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,LDR_Value);  //LED2
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,LDR_Value);  //LED3
			break;
		case 4:
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,LDR_Value);  //LED4
			break;
		case 5:
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,LDR_Value);  //LED5
			break;
		case 6:
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,LDR_Value);  //LED6
			break;
		case 7:
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,LDR_Value);  //LED7
			break;
		case 8:
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,LDR_Value); //LED8
			break;
		case 9:
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,LDR_Value); //LED9
			break;
	}
}

void PrintSegment(char digit_seg)
{
	switch(digit_seg)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_SET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_SET);
			break;
	}
}

void EnableSegment(char enable_seg)
{
	switch(enable_seg)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4, GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4, GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4, GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4, GPIO_PIN_RESET);
		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12, GPIO_PIN_RESET);
			break;
	}
}
void Seven_Segment_Time()
{
	if(Select_Segment == 0)
		{
			EnableSegment(1);
			PrintSegment(Second % 10);
			Select_Segment++;
		}
		else if(Select_Segment == 1)
		{
			EnableSegment(2);
			PrintSegment((Second / 10) % 10);
			Select_Segment++;
		}
		else if(Select_Segment == 2)
		{
			EnableSegment(3);
			PrintSegment(Minute % 10);
			Select_Segment++;
		}
		else if(Select_Segment == 3)
		{
			EnableSegment(4);
			PrintSegment((Minute / 10) % 10);
			Select_Segment = 0;
		}
}

void randing(uint8_t Rand_Number)
{
	uint8_t i, j, k;
	uint32_t hold;


	for(i=0;i<ARRAY_Number;i++)
	{
		Rand_Time_Array[i] = 0;
		Rand_Location_Array[i] = 0;
	}
	srand(counter);


	for(i=0;i<Rand_Number;i++)
	{
		Rand_Time_Array[i] = (rand() % TIME_Level) + 1;		
		Rand_Location_Array[i] = (rand() % 12) + 4;
	}


	for(j=0;j<Rand_Number-1;j++)
	{
		for(k=0;k<Rand_Number-1;k++)
		{
			if(Rand_Time_Array[k] > Rand_Time_Array[k+1])
			{
				hold = Rand_Time_Array[k];
				Rand_Time_Array[k] = Rand_Time_Array[k+1];
				Rand_Time_Array[k+1] = hold;
			}
		}
	}

}


void Speed_Machine(uint8_t fall_number)
{
	
	
	
	switch(state)
	{
		case Level_1:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,100 - LDR_Value);  //LED10
			break;
		case Level_2:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,100 - LDR_Value);  //LED8
			break;
		case Level_3:
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,100 - LDR_Value);  //LED6
			break;
		case Level_4:
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,100 - LDR_Value);  //LED4
			break;
		case Level_5:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,100 - LDR_Value);  //LED3
			break;
		case Level_6:
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,100 - LDR_Value);  //LED5
			break;
		case Level_7:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100 - LDR_Value);  //LED7
			break;
		case Level_8:
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100 - LDR_Value);  //LED9
			break;
		case Level_final:
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100 - LDR_Value);  //LED9
			break;
	}

}





void Display_Map()
{
	uint8_t column;
	uint8_t m, n = 0;

	setCursor(3,0);
	write(CC3);
	setCursor(16,0);
	write(CC4);

	setCursor(2,1);
	write(CC3);
	setCursor(17,1);
	write(CC4);

	setCursor(1,2);
	write(CC3);
	setCursor(18,2);
	write(CC4);

	setCursor(0,3);
	write(CC3);
	setCursor(19,3);
	write(CC4);

	for(column=1; column<19; column++)
	{
		setCursor(column,3);
		write(0x5F);
	}


	setCursor(Location_Macine,3);
	write(CC1);
	setCursor(Location_Macine+1,3);
	write(CC2);
	
	
	setCursor(0,1);
	sprintf(str,"%d",state);
	print(str);
	
	
	
	
	for(int i=0; i<Dike_Number; i++)
	{
		if((Dike_Array[i] != 0))
		{	
			if(Row_Fall[i] == 0)
			{
				setCursor(Column_Fall[i],Row_Fall[i]);
				write(0xFC);
			}
			else
			{
				//setCursor(Column_Fall[i],Row_Fall[i]-1);
				//write(' ');
				setCursor(Column_Fall[i],Row_Fall[i]-1);
				write(' ');
				setCursor(Column_Fall[i],Row_Fall[i]);
				write(0xFC);
			}
		}
	}
	
	
	
	
	/*
	if((Dike_Array[0]) || (Dike_Array[1]) || (Dike_Array[2]) || (Dike_Array[3]) ||
		 (Dike_Array[4]) || (Dike_Array[5]) || (Dike_Array[6]) || (Dike_Array[7]) ||
		 (Dike_Array[8]) || (Dike_Array[9]))
	{
		
		
		if(Row_Fall == 0)
		{
			setCursor(Column_Fall,Row_Fall);
			write(0xFC);
		}
		else if(Row_Fall == 1)
		{
			setCursor(Column_Fall,Row_Fall-1);
			write(' ');
			setCursor(Column_Fall,Row_Fall);
			write(0xFC);
		}
		else if(Row_Fall == 2)
		{
			setCursor(Column_Fall,Row_Fall-1);
			write(' ');
			setCursor(Column_Fall,Row_Fall);
			write(0xFC);
		}
		else if(Row_Fall == 3)
		{
			setCursor(Column_Fall,Row_Fall-1);
			write(' ');
			setCursor(Column_Fall,Row_Fall);
			write(0xFC);
		}
		
		
	}
	*/
	

	//if(display_var == 1) display();
	//else if(display_var == 0) noDisplay();

	display();

}




void All_Off()
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);  //LED10
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);  //LED8
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);  //LED6
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);  //LED4
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);  //LED3
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);  //LED5
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);  //LED7
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);  //LED9

	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);  //LED0
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);  //LED1
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);  //LED2
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);  //LED3
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);  //LED4
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);  //LED5
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);  //LED6
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);  //LED7
	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,0); //LED8
	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,0); //LED9
	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,100); //Test pin off Seven Segment
	//noDisplay();
	clear();
	//display();
	/*
	setCursor(0,0);
	print("                    ");
	setCursor(0,1);
	print("                    ");
	setCursor(0,2);
	print("                    ");
	setCursor(0,3);
	print("                    ");
	*/
}


void Led_Blink(uint8_t on_off_led)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,on_off_led);  //LED10
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,on_off_led);  //LED8
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,on_off_led);  //LED6
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,on_off_led);  //LED4
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,on_off_led);  //LED3
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,on_off_led);  //LED5
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,on_off_led);  //LED7
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,on_off_led);  //LED9

	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,on_off_led);  //LED0
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,on_off_led);  //LED1
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,on_off_led);  //LED2
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,on_off_led);  //LED3
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,on_off_led);  //LED4
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,on_off_led);  //LED5
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,on_off_led);  //LED6
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,on_off_led);  //LED7
	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,on_off_led); //LED8
	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,on_off_led); //LED9
}



void Lcd_Blink(uint8_t on_off_lcd)
{
	if(on_off_lcd == 1)
	{
		display();
	}
	else if(on_off_lcd == 0)
	{
		noDisplay();

	}
}



void Seven_Segment_Blink(uint8_t on_off_s7)
{
	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,on_off_s7); //Test pin off Seven Segment
}











/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC4)
	{
		LDR_Value = HAL_ADC_GetValue(&hadc4);
		LDR_Value = (LDR_Value * 100) / 63;

		HAL_ADC_Start_IT(&hadc4);
	}
	if(hadc->Instance == ADC1)
	{
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
	}
}
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance == TIM7)
	{

		Count_Level++;
		Time_Fall++;
		Count_Time++;
		Count_Turbo++;
		
		
		
		if(Count_Time == TIME_1S)
		{
			Count_Time = 0;
			Second++;
			if(Second == 60)
			{
				Minute++;
				Second = 0;
				if(Minute == 60)
				{
					Minute = 0;					
				}
			}
		}
		
		
		if(Count_Turbo == Turbo_TIME)
		{
			Count_Turbo = 0;
			Turbo++;
			if(Turbo == 10)
			{
				Turbo--;					
			}
		}
		
		
		
		Seven_Segment_Time();
		
		
		
		if(Dike_Number != state)
		{
			
			if(Count_Level == Rand_Time_Array[0])
			{
				Dike_Number = 1;
				Dike_Array[0] = 1;		
				Column_Fall[0] = Rand_Location_Array[0];
			}
			else if(Count_Level == Rand_Time_Array[1])
			{
				Dike_Number = 2;
				Dike_Array[1] = 1;				
				Column_Fall[1] = Rand_Location_Array[1];
			}
			else if(Count_Level == Rand_Time_Array[2])
			{
				Dike_Number = 3;
				Dike_Array[2] = 1;
				Column_Fall[2] = Rand_Location_Array[2];
			}
			else if(Count_Level == Rand_Time_Array[3])
			{
				Dike_Number = 4;
				Dike_Array[3] = 1;
				Column_Fall[3] = Rand_Location_Array[3];
			}
			else if(Count_Level == Rand_Time_Array[4])
			{
				Dike_Number = 5;
				Dike_Array[4] = 1;
				Column_Fall[4] = Rand_Location_Array[4];
			}
			else if(Count_Level == Rand_Time_Array[5])
			{
				Dike_Number = 6;
				Dike_Array[5] = 1;
				Column_Fall[5] = Rand_Location_Array[5];
			}
			else if(Count_Level == Rand_Time_Array[6])
			{
				Dike_Number = 7;
				Dike_Array[6] = 1;
				Column_Fall[6] = Rand_Location_Array[6];
			}
			else if(Count_Level == Rand_Time_Array[7])
			{
				Dike_Number = 8;
				Dike_Array[7] = 1;
				Column_Fall[7] = Rand_Location_Array[7];
			}
			else if(Count_Level == Rand_Time_Array[8])
			{
				Dike_Number = 9;
				Dike_Array[8] = 1;
				Column_Fall[8] = Rand_Location_Array[8];
			}
			else if(Count_Level == Rand_Time_Array[9])
			{
				Dike_Number = 10;
				Dike_Array[9] = 1;
				Column_Fall[9] = Rand_Location_Array[9];
			}
			
		}
		
		
		
		
		
		
		
	

		switch(state)
		{
			case Level_1:
			{
				if(Time_Fall == Time_Fall_Level_1)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
				}	
				break;
			}	
			case Level_2:
			{
				if(Time_Fall == Time_Fall_Level_2)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
				}	
				break;
			}
			case Level_3:
			{
				if(Time_Fall == Time_Fall_Level_3)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					if(Dike_Array[4] != 0) Row_Fall[4]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
					if(Row_Fall[4] == 4) Dike_Array[4] = 0;
				}	
				break;
			}
			case Level_4:
			{
				if(Time_Fall == Time_Fall_Level_4)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					if(Dike_Array[4] != 0) Row_Fall[4]++;
					if(Dike_Array[5] != 0) Row_Fall[5]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
					if(Row_Fall[4] == 4) Dike_Array[4] = 0;
					if(Row_Fall[5] == 4) Dike_Array[5] = 0;
				}	
				break;
			}
			case Level_5:
			{
				if(Time_Fall == Time_Fall_Level_5)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					if(Dike_Array[4] != 0) Row_Fall[4]++;
					if(Dike_Array[5] != 0) Row_Fall[5]++;
					if(Dike_Array[6] != 0) Row_Fall[6]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
					if(Row_Fall[4] == 4) Dike_Array[4] = 0;
					if(Row_Fall[5] == 4) Dike_Array[5] = 0;
					if(Row_Fall[6] == 4) Dike_Array[6] = 0;
				}	
				break;
			}
			case Level_6:
			{
				if(Time_Fall == Time_Fall_Level_6)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					if(Dike_Array[4] != 0) Row_Fall[4]++;
					if(Dike_Array[5] != 0) Row_Fall[5]++;
					if(Dike_Array[6] != 0) Row_Fall[6]++;
					if(Dike_Array[7] != 0) Row_Fall[7]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
					if(Row_Fall[4] == 4) Dike_Array[4] = 0;
					if(Row_Fall[5] == 4) Dike_Array[5] = 0;
					if(Row_Fall[6] == 4) Dike_Array[6] = 0;
					if(Row_Fall[7] == 4) Dike_Array[7] = 0;
				}	
				break;
			}
			case Level_7:
			{
				if(Time_Fall == Time_Fall_Level_7)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					if(Dike_Array[4] != 0) Row_Fall[4]++;
					if(Dike_Array[5] != 0) Row_Fall[5]++;
					if(Dike_Array[6] != 0) Row_Fall[6]++;
					if(Dike_Array[7] != 0) Row_Fall[7]++;
					if(Dike_Array[8] != 0) Row_Fall[8]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
					if(Row_Fall[4] == 4) Dike_Array[4] = 0;
					if(Row_Fall[5] == 4) Dike_Array[5] = 0;
					if(Row_Fall[6] == 4) Dike_Array[6] = 0;
					if(Row_Fall[7] == 4) Dike_Array[7] = 0;
					if(Row_Fall[8] == 4) Dike_Array[8] = 0;
				}
				break;
			}
			case Level_8:
			{
				if(Time_Fall == Time_Fall_Level_8)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					if(Dike_Array[4] != 0) Row_Fall[4]++;
					if(Dike_Array[5] != 0) Row_Fall[5]++;
					if(Dike_Array[6] != 0) Row_Fall[6]++;
					if(Dike_Array[7] != 0) Row_Fall[7]++;
					if(Dike_Array[8] != 0) Row_Fall[8]++;
					if(Dike_Array[9] != 0) Row_Fall[9]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
					if(Row_Fall[4] == 4) Dike_Array[4] = 0;
					if(Row_Fall[5] == 4) Dike_Array[5] = 0;
					if(Row_Fall[6] == 4) Dike_Array[6] = 0;
					if(Row_Fall[7] == 4) Dike_Array[7] = 0;
					if(Row_Fall[8] == 4) Dike_Array[8] = 0;
					if(Row_Fall[9] == 4) Dike_Array[9] = 0;
				}
				break;
			}
			case Level_final:
			{
				if(Time_Fall == Time_Fall_Level_final)
				{
					Time_Fall = 0;
					
					if(Dike_Array[0] != 0) Row_Fall[0]++;
					if(Dike_Array[1] != 0) Row_Fall[1]++;
					if(Dike_Array[2] != 0) Row_Fall[2]++;
					if(Dike_Array[3] != 0) Row_Fall[3]++;
					if(Dike_Array[4] != 0) Row_Fall[4]++;
					if(Dike_Array[5] != 0) Row_Fall[5]++;
					if(Dike_Array[6] != 0) Row_Fall[6]++;
					if(Dike_Array[7] != 0) Row_Fall[7]++;
					if(Dike_Array[8] != 0) Row_Fall[8]++;
					if(Dike_Array[9] != 0) Row_Fall[9]++;
					
					if(Row_Fall[0] == 4) Dike_Array[0] = 0;
					if(Row_Fall[1] == 4) Dike_Array[1] = 0;
					if(Row_Fall[2] == 4) Dike_Array[2] = 0;
					if(Row_Fall[3] == 4) Dike_Array[3] = 0;
					if(Row_Fall[4] == 4) Dike_Array[4] = 0;
					if(Row_Fall[5] == 4) Dike_Array[5] = 0;
					if(Row_Fall[6] == 4) Dike_Array[6] = 0;
					if(Row_Fall[7] == 4) Dike_Array[7] = 0;
					if(Row_Fall[8] == 4) Dike_Array[8] = 0;
					if(Row_Fall[9] == 4) Dike_Array[9] = 0;
				}
				break;
			}
		}
		
		
		
		
		
	

	

		if(Count_Level == TIME_Level)
		{
			
			Count_Level = 0;
			Time_Fall = 0;
			//Dike_Number = 0;
			for(int j=0;j<10;j++) Row_Fall[j] = 0;
			//for(int k=0;k<10;k++) Column_Fall[k] = 0;
			
			
			SW_Rand = 0;
			
			
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
	}
	
	
	if(htim->Instance == TIM6)
	{
		Turbo_Charge();
		
		
		switch(state)
		{
			case Blink:
			{
				if(counter % TIME_Blink == 0)
				{
					if(SW_Blink == 1)
					{
						Led_Blink(LDR_Value);
						Lcd_Blink(1);
						Seven_Segment_Blink(100 - LDR_Value);
						SW_Blink = 0;
					}
					else if(SW_Blink == 0)
					{
						Led_Blink(0);
						Lcd_Blink(0);
						Seven_Segment_Blink(100);
						SW_Blink = 1;
					}
				}
				break;
			}
			case Start:
			{
				All_Off();

				//HAL_TIM_Base_Stop_IT(&htim7);

				//setCursor(9,1);
				//print("**Death Race**");
				//write(0xff);
				//display();

				state = Level_1;
				break;
			}
			case Level_1:
			{
				if(SW_Rand == 0)
				{
					randing(3);					
					HAL_TIM_Base_Start_IT(&htim7);					
					//noDisplay();
					SW_Rand = 1;
				}	
				/*
				if(SW_Count_Level == 0)
				{
					Count_Level = 0;
					SW_Count_Level = 1;
				}
				*/

				
				//if((SW_Blink == 1) || (SW_Blink == 0) && ())
				Display_Map();
				Speed_Machine(state);
				
				

				//setCursor(8,0);
				//sprintf(str,"%d",Count_Level);
				//print(str);
				//display();


				break;
			}
			case Level_2:
			{
				if(SW_Rand == 0)
				{
					randing(4);							
					Count_Level = 0;
					//noDisplay();
					SW_Rand = 1;
				}				
				Display_Map();
				Speed_Machine(state);				
				break;
			}
			case Level_3:
			{
				if(SW_Rand == 0)
				{
					randing(5);
					Count_Level = 0;
					//noDisplay();
					SW_Rand = 1;
				}				
				Display_Map();
				Speed_Machine(state);
				break;
			}
			case Level_4:
			{
				if(SW_Rand == 0)
				{
					randing(6);
					Count_Level = 0;
					//noDisplay();
					SW_Rand = 1;
				}
				
				Display_Map();
				Speed_Machine(state);
				break;
			}
			case Level_5:
			{
				if(SW_Rand == 0)
				{
					randing(7);
					Count_Level = 0;
					//noDisplay();
					SW_Rand = 1;
				}
				Display_Map();
				Speed_Machine(state);
				break;
			}
			case Level_6:
			{
				if(SW_Rand == 0)
				{
					randing(8);
					Count_Level = 0;	
					//noDisplay();
					SW_Rand = 1;
				}
				
				Display_Map();
				Speed_Machine(state);
				break;
			}
			case Level_7:
			{
				if(SW_Rand == 0)
				{
					randing(9);
					Count_Level = 0;
					//noDisplay();
					SW_Rand = 1;
				}	
				Display_Map();
				Speed_Machine(state);
				break;
			}
			case Level_8:
			{
				if(SW_Rand == 0)
				{
					randing(10);
					Count_Level = 0;
					//noDisplay();
					SW_Rand = 1;
				}	
				Display_Map();
				Speed_Machine(state);
				break;
			}
			case Level_final:
			{
				if(SW_Rand == 0)
				{
					randing(10);
					Count_Level = 0;
					//noDisplay();
					SW_Rand = 1;
				}	
				Display_Map();
				Speed_Machine(state);
				break;
			}
			case Fail:
			{
				
				//Display_Map(1);
				break;
			}
		}


		if(counter == TIME_Blink) counter = 0;
		counter++;
	}
	
	HAL_TIM_Base_Start_IT(&htim6);
	
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  MX_ADC4_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */




	state = Blink;
	//Speed = state;


	LiquidCrystal(GPIOD, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14);
	begin(20,4);


	
	createChar(CC1, customChar1);
	createChar(CC2, customChar2);
	createChar(CC3, customChar3);
	createChar(CC4, customChar4);
	createChar(CC5, customChar5);
	

	noDisplay();
	setCursor(3,0);
	print("**Death Race**");
	setCursor(3,2);
	print("Ali  MohebiFar");
	setCursor(2,3);
	print("Majid AhmadPanah");



	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);  //LED10
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED8
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);  //LED6
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);  //LED4
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);  //LED3
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  //LED5
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //LED7
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);  //LED9

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);  //LED0
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);  //LED1
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);  //LED2
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);  //LED3
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);  //LED4
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);  //LED5
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);  //LED6
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);  //LED7
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED8
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED9

	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1); //Test pin off Seven Segment
	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,100);


	HAL_TIM_Base_Start_IT(&htim6);
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc4);

	//HAL_TIM_Base_Start_IT(&htim7);
	//HAL_TIM_Base_Stop_IT(&htim7);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC4 init function */
static void MX_ADC4_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config
    */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_6B;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 72;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 72;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 72;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim15);

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 72;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 99;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/* USB init function */
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE6 PE7 PE8 PE10
                           PE12 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC1 PC2 PC3
                           PC4 PC5 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF4 PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12
                           PB13 PB14 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
