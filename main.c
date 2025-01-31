/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

COM_InitTypeDef BspCOMInit;

PSSI_HandleTypeDef hpssi;
DMA_HandleTypeDef hdma_pssi;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
#define RIVI 128
#define SARAKE 128
static uint16_t kuva_buffer[RIVI][SARAKE]; //rxbufferi saapuvalle datalle
static uint32_t kuva_summa[10][RIVI]; //summamuuttuja rivinsummaukselle
volatile uint8_t Transfer_Ready = 0; //RDY lippu
volatile uint8_t rivisummat = 0; //Summatut rivit

volatile uint32_t start_time = 0; // Kokonais aloitusaika millisekunneissa
volatile uint32_t elapsed_time = 0; // Kulunut aika millisekunneissa
volatile uint16_t start_time_summaus; // Aloitusaika summaukselle
volatile uint16_t end_time_summaus;   // Lopetusaika summaukselle
//volatile uint16_t start_time_siirto; // Aloitusaika siirrolle
volatile uint16_t end_time_siirto; // Lopetusaika siirrolle



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_PSSI_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void HAL_PSSI_RxCpltCallback(PSSI_HandleTypeDef *hpssi);
static void RIVIEN_SUMMAUS(uint16_t kuva_buffer[RIVI][SARAKE], uint32_t sum[10][RIVI]);
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

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_PSSI_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_PSSI_Receive_DMA(&hpssi, (uint32_t*)kuva_buffer, sizeof(kuva_buffer)/4) != HAL_OK) //Käynnistetään PSSI vastaanotto sensorilta
  	           {
  	               Error_Handler(); // Virheen käsittely
  	           }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //GPIO_PinState rdy_state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
  HAL_TIM_Base_Start(&htim16); //Ajastin päälle summauksen ja siirron mittaamiseen
  start_time = __HAL_TIM_GET_COUNTER(&htim16); //Ajastin päälle koko prosessin mittaamiseen
  //start_time_siirto = __HAL_TIM_GET_COUNTER(&htim16);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //Asetetaan RDY-pinni ylös
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Aktivoituu kun kuva on saapunut
	  if (Transfer_Ready == 1) {
	          Transfer_Ready = 0;  // Nollataan odotustila
	          if (rivisummat < 10){ //Vastaanotetaan 10 kuvaa

	          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //asetetaan RDY nollaan
	          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	          }
	          else {
	        	  elapsed_time = (__HAL_TIM_GET_COUNTER(&htim16) - start_time); //Kun 10 kuvaa saapunut mitataan kulunut aika
	        	  float fps = (1000000.0 / elapsed_time) * 10; //lasketaan kuvien määrä sekunnissa
	        	  uint32_t tarkistus = 0;
	        	  for (int i = 0; i < 128; i++) {
	        	      tarkistus += kuva_summa[0][i]; // varmistus optimoinnille
	        	  }
	        	  printf(" 10 kuvaa vastaanotettu ajassa: %lu us\n\r FPS: %.2f\n\r Summauksen kesto: %u us\n\r PSSI siirron kesto: %u us\n\r tarkistus %lu\r\n\n", elapsed_time, fps, end_time_summaus, end_time_siirto, tarkistus);

	          }
	      }
	  	  //else {
	        //  printf("odotetaan dataa\n");
	          //HAL_Delay(1000);
	      //}
  //}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief PSSI Initialization Function
  * @param None
  * @retval None
  */
static void MX_PSSI_Init(void)
{

  /* USER CODE BEGIN PSSI_Init 0 */

  /* USER CODE END PSSI_Init 0 */

  /* USER CODE BEGIN PSSI_Init 1 */

  /* USER CODE END PSSI_Init 1 */
  hpssi.Instance = PSSI;
  hpssi.Init.DataWidth = HAL_PSSI_16BITS;
  hpssi.Init.BusWidth = HAL_PSSI_16LINES;
  hpssi.Init.ControlSignal = HAL_PSSI_DE_RDY_DISABLE;
  hpssi.Init.ClockPolarity = HAL_PSSI_FALLING_EDGE;
  hpssi.Init.DataEnablePolarity = HAL_PSSI_DEPOL_ACTIVE_LOW;
  hpssi.Init.ReadyPolarity = HAL_PSSI_RDYPOL_ACTIVE_HIGH;
  if (HAL_PSSI_Init(&hpssi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN PSSI_Init 2 */

  /* USER CODE END PSSI_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 274;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


static void RIVIEN_SUMMAUS(uint16_t kuva_buffer[RIVI][SARAKE], uint32_t sum[10][RIVI]) {
	for (int i = 0; i < RIVI; i++) {
	    uint32_t row_sum = 0;
	    for (int j = 0; j < SARAKE; j += 128) { // 128 summauksen putkitus
	        row_sum += kuva_buffer[i][j]   + kuva_buffer[i][j + 1]   + kuva_buffer[i][j + 2]   + kuva_buffer[i][j + 3]   +
	                   kuva_buffer[i][j + 4]   + kuva_buffer[i][j + 5]   + kuva_buffer[i][j + 6]   + kuva_buffer[i][j + 7]   +
	                   kuva_buffer[i][j + 8]   + kuva_buffer[i][j + 9]   + kuva_buffer[i][j + 10]  + kuva_buffer[i][j + 11]  +
	                   kuva_buffer[i][j + 12]  + kuva_buffer[i][j + 13]  + kuva_buffer[i][j + 14]  + kuva_buffer[i][j + 15]  +
	                   kuva_buffer[i][j + 16]  + kuva_buffer[i][j + 17]  + kuva_buffer[i][j + 18]  + kuva_buffer[i][j + 19]  +
	                   kuva_buffer[i][j + 20]  + kuva_buffer[i][j + 21]  + kuva_buffer[i][j + 22]  + kuva_buffer[i][j + 23]  +
	                   kuva_buffer[i][j + 24]  + kuva_buffer[i][j + 25]  + kuva_buffer[i][j + 26]  + kuva_buffer[i][j + 27]  +
	                   kuva_buffer[i][j + 28]  + kuva_buffer[i][j + 29]  + kuva_buffer[i][j + 30]  + kuva_buffer[i][j + 31]  +
	                   kuva_buffer[i][j + 32]  + kuva_buffer[i][j + 33]  + kuva_buffer[i][j + 34]  + kuva_buffer[i][j + 35]  +
	                   kuva_buffer[i][j + 36]  + kuva_buffer[i][j + 37]  + kuva_buffer[i][j + 38]  + kuva_buffer[i][j + 39]  +
	                   kuva_buffer[i][j + 40]  + kuva_buffer[i][j + 41]  + kuva_buffer[i][j + 42]  + kuva_buffer[i][j + 43]  +
	                   kuva_buffer[i][j + 44]  + kuva_buffer[i][j + 45]  + kuva_buffer[i][j + 46]  + kuva_buffer[i][j + 47]  +
	                   kuva_buffer[i][j + 48]  + kuva_buffer[i][j + 49]  + kuva_buffer[i][j + 50]  + kuva_buffer[i][j + 51]  +
	                   kuva_buffer[i][j + 52]  + kuva_buffer[i][j + 53]  + kuva_buffer[i][j + 54]  + kuva_buffer[i][j + 55]  +
	                   kuva_buffer[i][j + 56]  + kuva_buffer[i][j + 57]  + kuva_buffer[i][j + 58]  + kuva_buffer[i][j + 59]  +
	                   kuva_buffer[i][j + 60]  + kuva_buffer[i][j + 61]  + kuva_buffer[i][j + 62]  + kuva_buffer[i][j + 63]  +
	                   kuva_buffer[i][j + 64]  + kuva_buffer[i][j + 65]  + kuva_buffer[i][j + 66]  + kuva_buffer[i][j + 67]  +
	                   kuva_buffer[i][j + 68]  + kuva_buffer[i][j + 69]  + kuva_buffer[i][j + 70]  + kuva_buffer[i][j + 71]  +
	                   kuva_buffer[i][j + 72]  + kuva_buffer[i][j + 73]  + kuva_buffer[i][j + 74]  + kuva_buffer[i][j + 75]  +
	                   kuva_buffer[i][j + 76]  + kuva_buffer[i][j + 77]  + kuva_buffer[i][j + 78]  + kuva_buffer[i][j + 79]  +
	                   kuva_buffer[i][j + 80]  + kuva_buffer[i][j + 81]  + kuva_buffer[i][j + 82]  + kuva_buffer[i][j + 83]  +
	                   kuva_buffer[i][j + 84]  + kuva_buffer[i][j + 85]  + kuva_buffer[i][j + 86]  + kuva_buffer[i][j + 87]  +
	                   kuva_buffer[i][j + 88]  + kuva_buffer[i][j + 89]  + kuva_buffer[i][j + 90]  + kuva_buffer[i][j + 91]  +
	                   kuva_buffer[i][j + 92]  + kuva_buffer[i][j + 93]  + kuva_buffer[i][j + 94]  + kuva_buffer[i][j + 95]  +
	                   kuva_buffer[i][j + 96]  + kuva_buffer[i][j + 97]  + kuva_buffer[i][j + 98]  + kuva_buffer[i][j + 99]  +
	                   kuva_buffer[i][j + 100] + kuva_buffer[i][j + 101] + kuva_buffer[i][j + 102] + kuva_buffer[i][j + 103] +
	                   kuva_buffer[i][j + 104] + kuva_buffer[i][j + 105] + kuva_buffer[i][j + 106] + kuva_buffer[i][j + 107] +
	                   kuva_buffer[i][j + 108] + kuva_buffer[i][j + 109] + kuva_buffer[i][j + 110] + kuva_buffer[i][j + 111] +
	                   kuva_buffer[i][j + 112] + kuva_buffer[i][j + 113] + kuva_buffer[i][j + 114] + kuva_buffer[i][j + 115] +
	                   kuva_buffer[i][j + 116] + kuva_buffer[i][j + 117] + kuva_buffer[i][j + 118] + kuva_buffer[i][j + 119] +
	                   kuva_buffer[i][j + 120] + kuva_buffer[i][j + 121] + kuva_buffer[i][j + 122] + kuva_buffer[i][j + 123] +
	                   kuva_buffer[i][j + 124] + kuva_buffer[i][j + 125] + kuva_buffer[i][j + 126] + kuva_buffer[i][j + 127];
	    }
	    sum[rivisummat][i] = row_sum;
	}

    rivisummat++;

}


//keskeytyksen käsittelijä aktivoituu kun rxbufferi on täyttynyt
void HAL_PSSI_RxCpltCallback(PSSI_HandleTypeDef *hpssi)
{

	if (rivisummat < 1) {
	end_time_siirto = __HAL_TIM_GET_COUNTER(&htim16) - start_time;
	}

	start_time_summaus = __HAL_TIM_GET_COUNTER(&htim16);

	RIVIEN_SUMMAUS(kuva_buffer, kuva_summa); //summataan rivit

	end_time_summaus = __HAL_TIM_GET_COUNTER(&htim16) - start_time_summaus;

	Transfer_Ready++; //lippu ylös


}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
