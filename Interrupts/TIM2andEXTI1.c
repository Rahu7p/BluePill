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
#define BUTTON	( GPIOB->IDR & GPIO_IDR_IDR0 )
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
void USER_EXTI_Init(void);
void USER_GPIO_Init(void);
void USER_Debounce(void);
void USER_TIM2_Delay_1s(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM2_IRQHandler(void){
	if( ( TIM2->SR & TIM_SR_UIF ) == 0x01UL ){//		UEV-event causes the interrupt?
		GPIOC->ODR 	 = 	 GPIOC->ODR ^ GPIO_ODR_ODR13;//	toggle the User LED
		TIM2->SR	&=	~TIM_SR_UIF;//					UEV-event (overflow) cleared
	}
}

void EXTI1_IRQHandler(void){
	if( ( EXTI->PR & EXTI_PR_PR1 ) == 0x02UL ){//		EXTI1 line causes the interrupt?
		GPIOB->ODR 	 = 	 GPIOB->ODR ^ GPIO_ODR_ODR3;//	toggle the PB3 pin
		EXTI->PR	|=	~EXTI_PR_PR1;//					EXTI line request event cleared
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  USER_EXTI_Init();
  USER_GPIO_Init();
  USER_TIM2_Delay_1s();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if( BUTTON == 0 ){
		  USER_Debounce();
	  }
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
	//I/O port C clock enable
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN;
	//Timer 2 clock enable
	RCC->APB1ENR	|=	 RCC_APB1ENR_TIM2EN;
}

void USER_EXTI_Init(void){
	EXTI->IMR	|=	 EXTI_IMR_MR1;//	interrupt request line 1 enabled
	EXTI->RTSR	&=	 EXTI_RTSR_TR1;//	rising trigger disabled
	EXTI->FTSR	|=	 EXTI_FTSR_TR1;//	falling trigger enabled
}

void USER_GPIO_Init(void){
	//pin PA1 as input floating (for line 1 interrupt request)
	GPIOA->CRL	&=	~GPIO_CRL_CNF1_1 & ~GPIO_CRL_MODE1;
	GPIOA->CRL	|=	 GPIO_CRL_CNF1_0;
	NVIC_SetPriority(EXTI1_IRQn, 1);//						set 1-level priority
	NVIC_EnableIRQ(EXTI1_IRQn);//							enable EXTI1 vector handler

	//pin PB0 as input pull-up (to connect the push button)
	GPIOB->BSRR	 =   GPIO_BSRR_BS0;
	GPIOB->CRL	&=	~GPIO_CRL_CNF0_0 & ~GPIO_CRL_MODE0;
	GPIOB->CRL	|=	 GPIO_CRL_CNF0_1;

	GPIOB->BSRR	 =   GPIO_BSRR_BS1;//						PB1->1
	//pin PB1 as output push-pull, max speed 10MHz (button press event signal without debounce)
	GPIOB->CRL	&=	~GPIO_CRL_CNF1 & ~GPIO_CRL_MODE1_1;
	GPIOB->CRL 	|= 	 GPIO_CRL_MODE1_0;

	GPIOB->BSRR	 =   GPIO_BSRR_BR3;//						PB3->0
	//pin PB3 as output push-pull, max speed 10MHz (to connect LED2)
	GPIOB->CRL	&=	~GPIO_CRL_CNF3 & ~GPIO_CRL_MODE3_1;
	GPIOB->CRL 	|= 	 GPIO_CRL_MODE3_0;

	GPIOC->BSRR	 =   GPIO_BSRR_BS13;//						PC13->1, LED1 OFF
	//pin PC13 as output push-pull, max speed 10MHz
	GPIOC->CRH	&=	~GPIO_CRH_CNF13 & ~GPIO_CRH_MODE13_1;
	GPIOC->CRH 	|= 	 GPIO_CRH_MODE13_0;
}

void USER_Debounce(void){
	HAL_Delay( 10 );//					wait 10 ms
	if( BUTTON == 1 ){//				if not pressed (1), then is noise
		return;//						get out of the function
	}
	//if pressed (0)
	GPIOB->BSRR	 =   GPIO_BSRR_BR1;//	PB1->0 (signal without debounce)
	while( BUTTON == 0 ){//				wait until BUTTON is released
	//									empty while, only to test the condition
	}
	HAL_Delay( 10 );//					wait 10 ms
	//if released (1)
	GPIOB->BSRR	 =   GPIO_BSRR_BS1;//	PB1->1 (signal without debounce)
}

void USER_TIM2_Delay_1s(void){
	//Timer 2 slave mode control register
	TIM2->SMCR	&=	~TIM_SMCR_ECE//				external clock 2 mode disabled
				&	~TIM_SMCR_SMS;//			slave mode disabled / internal clock
	//Timer 2 control register 1
	TIM2->CR1	&=	~TIM_CR1_CMS//				edge-aligned mode (counts up or down depending on DIR)
				& 	~TIM_CR1_DIR//				upcounter
				&   ~TIM_CR1_URS//				to generate an overflow UEV-event by software
				& 	~TIM_CR1_UDIS//				overflow UEV-event enabled
				& 	~TIM_CR1_CEN;//				counter disabled
	TIM2->CR1	|=	 TIM_CR1_ARPE;//			to load ARR value only when an UEV-event is generated

	TIM2->PSC	 =	 1098U;//					prescaler for 1s
	TIM2->ARR	 =	 65513U;//					maximum count until overflows for 1s
	TIM2->EGR	|=	 TIM_EGR_UG;//				generate the UEV-event, reset the counter
	TIM2->SR	&=	~TIM_SR_UIF;//				overflow UEV-event cleared

	TIM2->DIER	|=	 TIM_DIER_UIE;//			update interrupt enable
	NVIC_SetPriority(TIM2_IRQn, 0);//			set 0-level (high) priority
	NVIC_EnableIRQ(TIM2_IRQn);//				enable TIM2 vector handler

	TIM2->CR1	|=	 TIM_CR1_CEN;//				counter enabled
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
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
