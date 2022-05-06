/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);
void USER_TIM2_PWM_Init(uint16_t duty);
void USER_TIM3_Delay_2s(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  float duty_cycle = 0.0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */
  USER_RCC_Init();
  USER_GPIO_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  USER_TIM2_PWM_Init( ( uint16_t )( ( TIM2->ARR * duty_cycle ) ) );
	  USER_TIM3_Delay_2s( );
	  duty_cycle = duty_cycle + 0.25;
	  if( duty_cycle == 1.25 )
		  duty_cycle = 0.0;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void USER_RCC_Init(void){
	//I/O port A clock enable
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPAEN;
	//Timer 2 and 3 clock enable
	RCC->APB1ENR	|=	 RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN;
}
void USER_GPIO_Init(void){
	//PA0 (TIM2_CH1) as alternate function push-pull, max speed 10MHz
	GPIOA->CRL	&=	~GPIO_CRL_CNF0_0 & ~GPIO_CRL_MODE0_1;
	GPIOA->CRL	|=	 GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0_0;
}
void USER_TIM2_PWM_Init(uint16_t duty){
	//Timer 2 slave mode control register
	TIM2->SMCR	&=	~TIM_SMCR_ECE//				external clock 2 mode disabled
				&	~TIM_SMCR_SMS;//			slave mode disabled / internal clock
	//Timer 2 control register 1
	TIM2->CR1	&=	~TIM_CR1_CMS//				edge-aligned mode (depending on DIR)
				& 	~TIM_CR1_DIR//				upcounter
				&   ~TIM_CR1_URS//				overflow UEV-event also by software
				& 	~TIM_CR1_UDIS//				overflow UEV-event enabled
				& 	~TIM_CR1_CEN;//				counter disabled
	TIM2->CR1	|=	 TIM_CR1_ARPE;//			to load ARR value only in UEV-event

	//Timer 2 capture/compare enable register
	TIM2->CCER	&=	~TIM_CCER_CC1P//			OC1 active high, polarity
				&	~TIM_CCER_CC1E;//			CC1 channel OFF, OC1 disabled
	//Timer 2 capture/compare mode register
	TIM2->CCMR1	&=	~TIM_CCMR1_OC1M_0//			PWM mode 1
				&	~TIM_CCMR1_CC1S;//			CC1 channel as output
	TIM2->CCMR1	|=	 TIM_CCMR1_OC1M_2//			PWM mode 1
				|	 TIM_CCMR1_OC1M_1//			PWM mode 1
				|	 TIM_CCMR1_OC1PE;//			to load CCR1 value only in UEV-event

	TIM2->CCR1	 =	 duty;//					duty cycle
	TIM2->PSC	 =	 1U;//						prescaler for 1kHz
	TIM2->ARR	 =	 35599U;//					maximum count until match for 1kHz
	TIM2->EGR	|=	 TIM_EGR_UG;//				UEV-event, resets the counter
	TIM2->SR	&=	~TIM_SR_UIF;//				UEV-event cleared
	TIM2->CR1	|=	 TIM_CR1_CEN;//				counter enabled
	while( ( TIM2->SR & TIM_SR_UIF ) == 0 ){//	wait until UEV-event
	//											empty, only test the condition
	}
	TIM2->CCER	|=	 TIM_CCER_CC1E;//			OC1 output enabled
}
void USER_TIM3_Delay_2s(void){
	//Timer 2 slave mode control register
	TIM3->SMCR	&=	~TIM_SMCR_ECE//				external clock 2 mode disabled
				&	~TIM_SMCR_SMS;//			slave mode disabled / internal clock
	//Timer 2 control register 1
	TIM3->CR1	&=	~TIM_CR1_CMS//				edge-aligned mode (depending on DIR)
				& 	~TIM_CR1_DIR//				upcounter
				&   ~TIM_CR1_URS//				overflow UEV-event also by software
				& 	~TIM_CR1_UDIS//				overflow UEV-event enabled
				& 	~TIM_CR1_CEN;//				counter disabled
	TIM3->CR1	|=	 TIM_CR1_ARPE;//			to load ARR value only in UEV-event
	TIM3->PSC	 =	 1098U * 2;//				prescaler for 2s
	TIM3->ARR	 =	 65513U;//					maximum count until overflows for 2s
	TIM3->EGR	|=	 TIM_EGR_UG;//				UEV-event, reset the counter
	TIM3->SR	&=	~TIM_SR_UIF;//				overflow UEV-event cleared
	TIM3->CR1	|=	 TIM_CR1_CEN;//				counter enabled
	while( ( TIM3->SR & TIM_SR_UIF ) == 0 ){//	wait until overflow UEV-event
	//											empty, only test the condition
	}
	TIM3->CR1	&=	~TIM_CR1_CEN;//				counter disabled
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of erropr occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
