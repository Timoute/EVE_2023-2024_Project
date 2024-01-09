/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "mpu6050.h"
#include "string.h"
#include "stm32g4xx_hal.h"

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t TX_Buffer[] = "A" ; // DATA to send
uint8_t uartTxBuffer[32];
MPU6050_t MPU6050;
#define ADDRESS_WRITE   0x52
#define ADDRESS_READ    0x53
#define MPU6050_I2C_ADDR 0x69

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

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

//	GPIO_InitTypeDef GPIO_InitStruct;
//
//	// Configurer la broche comme une entrée avec résistance de pull-up
//	GPIO_InitStruct.Pin = GPIO_PIN_0;  // Exemple : utilisez le numéro de broche approprié
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // Mode d'interruption déclenché sur front montant
//	GPIO_InitStruct.Pull = GPIO_PULLUP;  // Résistance de tirage pull-up
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Initialisez la broche avec la configuration
//
//	// Configurer la priorité de l'interruption
//	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);  // Epriorité de l'interruption
//	HAL_NVIC_EnableIRQ(EXTI0_IRQn);  // interruption pour la broche EXTI0
//

	uint8_t   return_value;
	uint8_t buf[12];
	int sizePrint,sizePrint0,sizePrint1,sizePrint2,sizePrint3,sizePrint4,sizePrint5,sizePrint6,sizePrint7,donnee;
	HAL_StatusTypeDef erreur_transmit,erreur_master_receive;
	unsigned char UcData[8]={0,0,0,0,0,0,0,0};
	uint8_t received_data[16]; // Pour stocker les 5 octets reçus
	HAL_StatusTypeDef status;
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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_I2C2_Init();
  MPU6050_Init(&hi2c2);
  /* USER CODE BEGIN 2 */

//  HAL_I2C_Master_Transmit_IT(&hi2c1,20,TX_Buffer,1); //Sending in Interrupt mode
  HAL_Delay(100);
  sizePrint = snprintf(uartTxBuffer, 32, "Starting\r\n");
  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {//memset(received_data, 0, 5);

	  /*read_VL53L1X();*/
	  HAL_Delay (1000);
	  MPU6050_Read_All(&hi2c2, &MPU6050);
	  HAL_Delay (300);
	  char uartBufferAccel[100];
	  char uartBufferGyro[100];
	  char uartBufferTemp[100];

	  /*snprintf(uartBuffer, sizeof(uartBuffer), "Ax: %f\n"
	                                   "Ay: %f\n"
	                                   "Az: %f\n"
	  				 	 	 	 	   "Temperature: %f\n"
	                                   "Gx: %f\n"
	                                   "Gy: %f\n"
	                                   "Gz: %f\n"
	                                  , MPU6050.Ax, MPU6050.Ay, MPU6050.Az, MPU6050.Temperature, MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);*/

	  MPU6050_Read_Accel(&hi2c2, &MPU6050);
	  snprintf(uartBufferAccel, sizeof(uartBufferAccel),"Ax: %f\r\nAy: %f\r\nAz: %f\r\n",MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBufferAccel, strlen(uartBufferAccel), HAL_MAX_DELAY);
	  HAL_Delay (200);

	  MPU6050_Read_Gyro(&hi2c2, &MPU6050);
	  snprintf(uartBufferGyro, sizeof(uartBufferGyro), "Gx: %f\r\nGy: %f\r\nGz: %f\r\n",MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBufferGyro, strlen(uartBufferGyro), HAL_MAX_DELAY);
	  HAL_Delay (200);

	  MPU6050_Read_Temp(&hi2c2, &MPU6050);
	  snprintf(uartBufferTemp, sizeof(uartBufferTemp),"Temperature: %f\r\n",MPU6050.Temperature);
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBufferTemp, strlen(uartBufferTemp), HAL_MAX_DELAY);
	  HAL_Delay (200);

	  MPU6050_Read_All(&hi2c2, &MPU6050);
	  snprintf(uartBufferTemp, sizeof(uartBufferTemp),"Pitch: %f\r\nRoll: %f\r\n",MPU6050.KalmanAngleY,MPU6050.KalmanAngleX);
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBufferTemp, strlen(uartBufferTemp), HAL_MAX_DELAY);
	  HAL_Delay (200);



	  calculateVelocities(&MPU6050);
	  char uartBufferVelocities[100];
	  snprintf(uartBufferVelocities, sizeof(uartBufferVelocities), "Vx: %f\r\nVy: %f\r\nVz: %f\r\n", velocityX, velocityY, velocityZ);
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBufferVelocities, strlen(uartBufferVelocities), HAL_MAX_DELAY);
	  HAL_Delay(200);







	  /*return_value = HAL_I2C_IsDeviceReady(&hi2c1, 0x52<<1, 1, 100);

	  const char *message;
	  if (return_value == 1)
	  {
	      message = "Le périphérique est prêt \r\n";
	  }
	  else
	  {
	      message = "Le périphérique n'est pas prêt \r\n";
	  }
	  sizePrint = snprintf(uartTxBuffer, 32, "Return value : %d\r\n", return_value);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);

	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)message, strlen(message), 100);
	  /*return_value = HAL_I2C_IsDeviceReady(&hi2c1, 0x52<<1, 1, 100);
	  sizePrint = snprintf(uartTxBuffer, 32, "Return value : %d\r\n", return_value);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);*/

	  HAL_Delay(500);

	  /*// Démarrer la communication I2C
	  buf[0]=0x00;
	  status = HAL_I2C_Master_Transmit(&hi2c1, ADDRESS_READ << 1,buf, 1, HAL_MAX_DELAY);
	  if (status != HAL_OK) {// Gestion d'erreur
		  erreur_transmit=snprintf(uartTxBuffer, 32, "Erreur Transmit: %d\r\n",status);
		  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, erreur_transmit, 100);


	  }

	  // Envoyer l'adresse du périphérique en lecture
	  status = HAL_I2C_Master_Receive(&hi2c1, (ADDRESS_READ << 1), received_data, sizeof(received_data), HAL_MAX_DELAY);
	  if (status != HAL_OK) {
	      // Gestion d'erreur
		  erreur_master_receive=snprintf(uartTxBuffer, 32, "Erreur Master_Receive: %d\r\n",status);
		  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, erreur_master_receive, 100);
	  }
	  for (int i=0;i<16;i++) {
	      sizePrint = snprintf(uartTxBuffer, 32, "Element %d : %d\r\n", i, received_data[i]);
	      HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);
	  }


	  HAL_Delay(1000);*/

	  /*for (int i=0;i<5;i++) {
		  received_data[i]=0;
	  }*/






	  /*HAL_I2C_Mem_Read(&hi2c1, 0x52<<1, 0x00,1,&UcData[0],  2, 100);
	  HAL_I2C_Mem_Read(&hi2c1, 0x52<<1, 0x00,1,&UcData[1],  2, 100);
	  HAL_I2C_Mem_Read(&hi2c1, 0x52<<1, 0x00,1,&UcData[2],  2, 100);
	  HAL_I2C_Mem_Read(&hi2c1, 0x52<<1, 0x00,1,&UcData[3],  2, 100);

	  HAL_I2C_Mem_Read(&hi2c1, 0x53<<1, 0x00,1,&UcData[4],  2, 100);
	  HAL_I2C_Mem_Read(&hi2c1, 0x53<<1, 0x00,1,&UcData[5],  2, 100);
	  HAL_I2C_Mem_Read(&hi2c1, 0x53<<1, 0x00,1,&UcData[6],  2, 100);
	  HAL_I2C_Mem_Read(&hi2c1, 0x53<<1, 0x00,1,&UcData[7],  2, 100);

	  sizePrint0 = snprintf(uartTxBuffer, 32, "Bit 0 : %d\r\n", UcData[0]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint0, 100);
	  sizePrint1 = snprintf(uartTxBuffer, 32, "Bit 1 : %d\r\n", UcData[1]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint1, 100);
	  sizePrint2 = snprintf(uartTxBuffer, 32, "Bit 2 : %d\r\n", UcData[2]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint2, 100);
	  sizePrint3 = snprintf(uartTxBuffer, 32, "Bit 3 : %d\r\n", UcData[3]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint3, 100);
	  sizePrint4 = snprintf(uartTxBuffer, 32, "Bit 4 : %d\r\n", UcData[4]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint4, 100);
	  sizePrint5 = snprintf(uartTxBuffer, 32, "Bit 5 : %d\r\n", UcData[5]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint5, 100);
	  sizePrint6 = snprintf(uartTxBuffer, 32, "Bit 6 : %d\r\n", UcData[6]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint6, 100);
	  sizePrint7 = snprintf(uartTxBuffer, 32, "Bit 7 : %d\r\n\n", UcData[7]);
	  HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint7, 100);
	  HAL_Delay(1000);*/

	  /*strcpy((char*)buf,"Hello\r\n");
	  HAL_UART_Transmit(&hlpuart1,buf,strlen((char*)buf),HAL_MAX_DELAY);*/
	  HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    }


    /* USER CODE BEGIN 3 */

void ReadMPU6050Data(void) {
	uint8_t buffer[6];
	int sizePrint;
	uint8_t uartTxBuffer[32];
	uint8_t received_data[16]; // Pour stocker les 5 octets reçus
	uint8_t buf[12];


    // Envoyer la commande de début de communication au MPU-60X0
	buf[0]=0x00;
    HAL_I2C_Master_Transmit(&hi2c2, MPU6050_I2C_ADDR << 1, buf, 1, HAL_MAX_DELAY);
    // Lire les données du MPU-60X0
    HAL_I2C_Master_Receive(&hi2c2, MPU6050_I2C_ADDR, buffer, 6, HAL_MAX_DELAY);
	for (int i=0;i<sizeof(buffer);i++) {
	    sizePrint = snprintf(uartTxBuffer, 32, "Element gyro %d : %d\r\n", i, buffer[i]);
	    HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);

    // Maintenant, le buffer contient les données lues du MPU-60X0
	}
}
void read_VL53L1X(void){
	uint8_t   return_value;
	uint8_t buf[12];
	int sizePrint,sizePrint0,sizePrint1,sizePrint2,sizePrint3,sizePrint4,sizePrint5,sizePrint6,sizePrint7,donnee;
	HAL_StatusTypeDef erreur_transmit,erreur_master_receive;
	unsigned char UcData[8]={0,0,0,0,0,0,0,0};
	uint8_t received_data[16]; // Pour stocker les 5 octets reçus

	HAL_StatusTypeDef status;
	uint8_t TX_Buffer[] = "A" ; // DATA to send
	uint8_t uartTxBuffer[32];
	return_value = HAL_I2C_IsDeviceReady(&hi2c1, 0x52<<1, 1, 100);

	const char *message;
	if (return_value == 1)
	{
	    message = "Le tof est prêt \r\n";
	}
	else
	{
	    message = "Le tof n'est pas prêt \r\n";
	}
	sizePrint = snprintf(uartTxBuffer, 32, "Return value : %d\r\n", return_value);
	HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)message, strlen(message), 100);
	return_value = HAL_I2C_IsDeviceReady(&hi2c1, 0x52<<1, 1, 100);
	sizePrint = snprintf(uartTxBuffer, 32, "Return value : %d\r\n", return_value);
	HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);

	HAL_Delay(500);

	  // Démarrer la communication I2C
	buf[0]=0x00;
	status = HAL_I2C_Master_Transmit(&hi2c1, ADDRESS_READ << 1,buf, 1, HAL_MAX_DELAY);
	if (status != HAL_OK) {// Gestion d'erreur
		erreur_transmit=snprintf(uartTxBuffer, 32, "Erreur Transmit: %d\r\n",status);
		HAL_UART_Transmit(&hlpuart1, uartTxBuffer, erreur_transmit, 100);


	}

	  // Envoyer l'adresse du périphérique en lecture
	status = HAL_I2C_Master_Receive(&hi2c1, (ADDRESS_READ << 1), received_data, sizeof(received_data), HAL_MAX_DELAY);
	if (status != HAL_OK) {
	    // Gestion d'erreur
		erreur_master_receive=snprintf(uartTxBuffer, 32, "Erreur Master_Receive: %d\r\n",status);
		HAL_UART_Transmit(&hlpuart1, uartTxBuffer, erreur_master_receive, 100);
	}
	for (int i=0;i<16;i++) {
	    sizePrint = snprintf(uartTxBuffer, 32, "Element tof %d : %d\r\n", i, received_data[i]);
	    HAL_UART_Transmit(&hlpuart1, uartTxBuffer, sizePrint, 100);
	}


	HAL_Delay(1000);
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10802D9B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
