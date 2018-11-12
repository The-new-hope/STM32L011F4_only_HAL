/* Includes ------------------------------------------------------------------*/
#include "header.h"
/* Private variables ---------------------------------------------------------*/
BMP280_HandleTypedef bmp280;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
PWR_PVDTypeDef sConfigPVD;
IWDG_HandleTypeDef hiwdg;
/* Private variables ---------------------------------------------------------*/
float pressure, temperature, humidity;

//uint8_t buffer_RX[RF_DATA_SIZE];
uint8_t buffer_TX[RF_DATA_SIZE]; ////
uint8_t status_TX = 0;
uint8_t status_RX = 0;

uint8_t Tcounter = 0;
uint8_t Tcounter1 = 0;

uint16_t size_UART;
uint8_t Data[100];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
	Tcounter ++;
  Tcounter1 ++;
}

void PVD_IRQHandler(void)
{
  HAL_PWR_PVD_IRQHandler();
	buffer_TX[0] |= (1<<3);	
}

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
	MX_NVIC_Init();
  MX_IWDG_Init();	
	HAL_UART_MspInit(&huart2);

	HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);

	sConfigPVD.PVDLevel = Power_Level;
  sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING;
  HAL_PWR_ConfigPVD(&sConfigPVD);	
	
  HAL_PWR_EnablePVD();	
	
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	HAL_Delay(100);
	Conf_NRF_Rx();
	HAL_Delay(100);
	if (!bmp280_init(&bmp280, &bmp280.params)) {
		buffer_TX[0] |= (1<<2);																														// если датчик не отвечает
	}else{																																							// то датчик неисправен
		buffer_TX[0] &= ~(1<<2);																													//
	}
//	bool bme280p = bmp280.id == BME280_CHIP_ID;
//	size_UART = sprintf((char *)Data, "Sensor found %s\n\r", bme280p ? "BME280" : "BMP280");
//	HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
//		if (Tcounter1 >= 0) {//TIME_SENDING) {		//  аждые 5 сек // „итаем новые данные
//			Tcounter1 = 0;
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){																			// „итаем датчик дожд€
				buffer_TX[0] |=(1<<1);																														// есть дождь
			}	else {																																						//
				buffer_TX[0] &=~(1<<1);																														// нет дожд€
			}																																										//		
			if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {							// „итаем датчик погоды
				buffer_TX[0] |= (1<<2);																														// если датчик не отвечает
			}else{																																							// то датчик неисправен
				buffer_TX[0] &= ~(1<<2);																													//
			}
			if (pressure == 0){																																	// если измеренное давление
				buffer_TX[0] |= (1<<2);																														// равно 0 
			}else{																																							// то датчик неисправен
				buffer_TX[0] &= ~(1<<2);																													//
			}			
			uint16_t rezAtmPressureGPa_uint =  (uint16_t)pressure;
			uint16_t rezHumidity_uint =        (uint16_t)(humidity * 10);
			int16_t rezTemperature_int = (int16_t)(temperature * 10);			
			uint16_t rezAtmPressure_uint = (uint16_t)pressure * 0.75;
				
//			size_UART = sprintf((char *)Data,"Pressure: %.2f GPa, Temperature: %.2f C, Humidity: %.2f\n\r",
//					pressure, temperature, humidity);
//			HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		
			//_______________________________________
			//є байта  |    описание   buffer_TX[]   |
			//_________|_____________________________|  
			//   0     |    статус 
			//   1     |    температура int16_t - LSB 
			//   2     |    температура int16_t - MSB 
			//   3     |    влажность uint16_t   - LSB 
			//   4     |    влажность uint16_t   - MSB 
			//   5     |    давление, мм. рт. ст uint16_t - LSB 
			//   6     |    давление, мм. рт. ст uint16_t - MSB 
			//   7     |    давление, гѕа uint16_t - LSB 
			//   8     |    давление, гѕа uint16_t - MSB      		
			/*у нас нульовий байт статус
					перший б≥т в 1 - Ї дощ. 
					перший б≥т в 0 - нема дощу
					другий б≥т в 1 - несправн≥сть датчика погоди
					другий б≥т в 0 - все ок
					трет≥й б≥т в 1 - напруга на контроллер≥ менше 2.5 вольт
					трет≥й б≥т в 0 - напруга ќ */			
			buffer_TX[1]=rezTemperature_int & 0xFF;	// LSB
			buffer_TX[2]=rezTemperature_int >> 8;		// MSB
			buffer_TX[3]=rezHumidity_uint & 0xFF;
			buffer_TX[4]=rezHumidity_uint >> 8;
			buffer_TX[5]=rezAtmPressure_uint & 0xFF;
			buffer_TX[6]=rezAtmPressure_uint >> 8;
			buffer_TX[7]=rezAtmPressureGPa_uint & 0xFF;
			buffer_TX[8]=rezAtmPressureGPa_uint >> 8;
		
			send_data_NRF(buffer_TX,RF_DATA_SIZE);
			if (status_TX ==1){
				status_TX = 0;			
			}//	else {
			//							size_UART = sprintf((char *)Data, "Transmit Bad!!!\n\r");/////////////////////////////////////строка дл€ отладки///////////////////////////////////////////////
			//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		/////////////////////////////////////строка дл€ отладки////////////////////////////////////////////////
			//}			
			HAL_PWR_EnterSTANDBYMode();
//		}	
		
		
		
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	
	RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//	RCC_PeriphCLKInitTypeDef PeriphClkInit;
    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  __HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_PVD_EXTI_ENABLE_IT();
    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;//RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;//RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);	
	
	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PVD_IRQn);	
}



/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
//	CSN1;
//	CE0;	
	
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}




/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



}

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
