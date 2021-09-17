/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
char buffrec[6];

int ZDA=1;
int LIN=0;
int LEW=0;
int PRA=0;
int CZ1=0;
int CZ2=0;
int CZ3=0;
int CZ4=0;
int CZ5=0;
int CZ6=0;
int CZ7=0;

int KP=90;
int KI=17;
int KD=37;
int PREDKOSC=13;

float E=0;
float Eold=0;
float I =0;
float D =0;
int DOCELOWA=0;
int ZMIANA=0;
float POMOCNICZE=0;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
void ledtoggle()
{
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}	

void ledset()
{
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
}

void ledreset()
{
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
}

void ledblink()
{
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
HAL_Delay(200);
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
HAL_Delay(200);
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
HAL_Delay(200);
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
HAL_Delay(200);
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
HAL_Delay(200);
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
HAL_Delay(200);
HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
HAL_Delay(200);
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);	
}
 
void PWML(int wypa)
{
TIM1->CCR1=wypa; // Wpisanie wartosci wypelnienia PWMA
}

void PWMP(int wypb)
{
TIM4->CCR3=wypb; // Wpisanie wartosci wypelnienia PWMB
}

void PWMLstop()
{
TIM1->CCR1=0;
}

void PWMPstop()
{
TIM4->CCR3=0;
} 

void PWMLprzod()
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);  // AIN1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET); // AIN2
}

void PWMPprzod()
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); // BIN1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET); // BIN2
}

void PWMLtyl()
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);  // AIN1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); // AIN2
}

void PWMPtyl()
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); // BIN1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); // BIN2
}

void PWMLwolnystop()
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);  // AIN1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); // AIN2
}

void PWMPwolnystop()
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); // BIN1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); // BIN2
}   

void JAZDA()
{
if (LEW > 0 )
{
	PWMLprzod();
	PWML(LEW);
}
else if (LEW == 0 )
{
	PWMLstop();
}
else if (LEW < 0 )
{
	PWMLtyl();
	PWML(0 - LEW);
}
if (PRA > 0 )
{
	PWMPprzod();
	PWMP(PRA);
}
else if (PRA == 0 )
{
	PWMPstop();
}
else if (PRA < 0 )
{
	PWMPtyl();
	PWMP(0 - PRA);
}
} 

void STAN ()
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))  
   {
    CZ1=0;
   }
	 else 
	 {
		CZ1=1;
	 }
	 
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))  
   {
    CZ2=0;
   }
	 else 
	 {
		CZ2=1;
	 }
	 
	 if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))  
   {
    CZ3=0;
   }
	 else 
	 {
		CZ3=1;
	 }
	 
	 if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))  
   {
    CZ4=0;
   }
	 else 
	 {
		CZ4=1;
	 }
	 
	 if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))  
   {
    CZ5=0;
   }
	 else 
	 {
		CZ5=1;
	 }
	 
	 if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))  
   {
    CZ6=0;
   }
	 else 
	 {
		CZ6=1;
	 }
	 
	 if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))  
   {
    CZ7=0;
   }
	 else 
	 {
		CZ7=1;
	 }
}

void UCHYB ()
{
		POMOCNICZE = CZ5*1 + CZ6*2 + CZ7*3 - CZ1*3 - CZ2*2 - CZ3*1 ;
		POMOCNICZE = POMOCNICZE * 0.1;
		E = DOCELOWA - POMOCNICZE;
		D=E-Eold;
		I=I+E;
		Eold=E;
}

void LF ()
{
	ZMIANA=KP*E+KI*I*0.0001+KD*D;
		LEW=PREDKOSC-ZMIANA;
		PRA=PREDKOSC+ZMIANA;
}

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); // uruchomienie PWMA
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); // uruchomienie PWMB
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);  // STBY stan 1
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		if (LIN ==0 && ZDA ==1)
		{
		JAZDA();
		}
		else if (LIN ==1 && ZDA ==0)
		{
			STAN();
			UCHYB();
			LF();
			JAZDA();
			
			
			
			
			
			
			
			
			
	  
		}
		else 
		{
		PWMLstop();
		PWMPstop();
		}
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
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

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 
                           PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB5 PB6 
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
