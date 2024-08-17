/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "gps_neo6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADXL345_ADDRESS 0x53<<1 // ADXL345 I2C address shifted left by 1 bit
#define FALL_THRESHOLD_MIN 0.7
#define FALL_THRESHOLD_MAX 2.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId ReadAcceloTaskHandle;
osThreadId LedTaskHandle;
osThreadId ReadLocationHandle;
osThreadId DetectChuteTaskHandle;
osThreadId GSMTaskHandle;
osThreadId AlertButtonTaskHandle;
osMessageQId AccelQueueHandle;
osMessageQId DetectChuteQueueHandle;
osMessageQId GPSQueueHandle;
osMessageQId ButtonQueueHandle;
/* USER CODE BEGIN PV */
uint8_t Message[64];
uint8_t MessageLength;
NEO6_State GpsState;
char phone_number[] = "+21658944737";
volatile uint8_t rx_data;
uint8_t rx_buffer[256];
volatile uint16_t rx_index = 0;
volatile uint8_t new_sms_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
void ReadAcceleration(void const * argument);
void led(void const * argument);
void ReadLocalion(void const * argument);
void DetectChute(void const * argument);
void GSM(void const * argument);
void AlertButton(void const * argument);

/* USER CODE BEGIN PFP */
void ADXL345_Init();
void adxl_write(uint8_t reg,uint8_t value);
void SendSms(char* message);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GSM_Init(void);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  ADXL345_Init();
  NEO6_Init(&GpsState, &huart1);
  GSM_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of AccelQueue */
  osMessageQDef(AccelQueue, 1, uint16_t);
  AccelQueueHandle = osMessageCreate(osMessageQ(AccelQueue), NULL);

  /* definition and creation of DetectChuteQueue */
  osMessageQDef(DetectChuteQueue, 1, uint8_t);
  DetectChuteQueueHandle = osMessageCreate(osMessageQ(DetectChuteQueue), NULL);

  /* definition and creation of GPSQueue */
  osMessageQDef(GPSQueue, 1, uint8_t*);
  GPSQueueHandle = osMessageCreate(osMessageQ(GPSQueue), NULL);

  /* definition and creation of ButtonQueue */
  osMessageQDef(ButtonQueue, 1, uint8_t);
  ButtonQueueHandle = osMessageCreate(osMessageQ(ButtonQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ReadAcceloTask */
  osThreadDef(ReadAcceloTask, ReadAcceleration, osPriorityLow, 0, 128);
  ReadAcceloTaskHandle = osThreadCreate(osThread(ReadAcceloTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, led, osPriorityLow, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of ReadLocation */
  osThreadDef(ReadLocation, ReadLocalion, osPriorityLow, 0, 128);
  ReadLocationHandle = osThreadCreate(osThread(ReadLocation), NULL);

  /* definition and creation of DetectChuteTask */
  osThreadDef(DetectChuteTask, DetectChute, osPriorityLow, 0, 512);
  DetectChuteTaskHandle = osThreadCreate(osThread(DetectChuteTask), NULL);

  /* definition and creation of GSMTask */
  osThreadDef(GSMTask, GSM, osPriorityLow, 0, 128);
  GSMTaskHandle = osThreadCreate(osThread(GSMTask), NULL);

  /* definition and creation of AlertButtonTask */
  osThreadDef(AlertButtonTask, AlertButton, osPriorityLow, 0, 128);
  AlertButtonTaskHandle = osThreadCreate(osThread(AlertButtonTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  // Start UART receive interrupt
      HAL_UART_Receive_IT(&huart6, &rx_data, 1);
  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void adxl_write(uint8_t reg,uint8_t value){
	uint8_t data[2];
	data[0] = reg; // Address of the POWER_CTL register
	data[1] = value; // Enable the sensor
	HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDRESS, data, 2, 200);
}

void ADXL345_Init()
{
	  adxl_write(0x31,0x03); // Configure the measurement range to ±16+g
	  // Enable the ADXL345 accelerometer by writing 0x08 to the POWER_CTL register (Address: 0x2D)
	  adxl_write(0x2d,0x00);
	  adxl_write(0x2d,0x08);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == GpsState.neo6_huart)
	{
		NEO6_ReceiveUartChar(&GpsState);
	}
	else if (huart->Instance == USART6)
    {
        // Store received data into buffer
        rx_buffer[rx_index++] = rx_data;

        // Check for end of message (e.g., newline character)
        if (rx_data == '\n')
        {
            rx_buffer[rx_index] = '\0'; // Null-terminate the buffer
            rx_index = 0; // Reset index for next message
            new_sms_received = 1; // Set flag to indicate a new message received
        }

        // Continue receiving data
        HAL_UART_Receive_IT(&huart6, &rx_data, 1);
    }
}

void GSM_Init(void)
{
	char msg[50];
	char cnmi_cmd[] = "AT+CNMI=2,2,0,0,0\r\n";
	char text_mode_cmd[] = "AT+CMGF=1\r\n";
	char init_cmd[] = "AT\r\n";

	sprintf(msg,"\r\n Initializing the GSM module\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
    HAL_UART_Transmit(&huart6, (uint8_t*)init_cmd, strlen(init_cmd), 1000);
    HAL_Delay(1000);

    // Set SMS text mode
    HAL_UART_Transmit(&huart6, (uint8_t*)text_mode_cmd, strlen(text_mode_cmd), 1000);
    HAL_Delay(1000);

    // Enable new message indications
    HAL_UART_Transmit(&huart6, (uint8_t*)cnmi_cmd, strlen(cnmi_cmd), 1000);
    HAL_Delay(1000);

}

void SendSms(char* message){
	char cmd[256];

	// Send SMS command sequence
	        sprintf(cmd, "AT+CMGF=1\r\n");
	        HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), 1000);
	        vTaskDelay(1000/portTICK_RATE_MS);

	        sprintf(cmd, "AT+CMGS=\"%s\"\r\n", phone_number);
	        HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), 1000);
	        vTaskDelay(1000/portTICK_RATE_MS);

	        HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), 1000);
	        vTaskDelay(1000/portTICK_RATE_MS);

	        // End of message character (Ctrl+Z or 0x1A in ASCII)
	        uint8_t end_char = 0x1A;
	        HAL_UART_Transmit(&huart6, &end_char, 1, 1000);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ReadAcceleration */
/**
  * @brief  Function implementing the ReadAcceleration thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadAcceleration */
void ReadAcceleration(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t data_rec[6];
	int16_t x,y,z;
	float xg,yg,zg,a;
  /* Infinite loop */
  for(;;)
  {
	  // Read 6 bytes of data starting from the DATAX0 register (Address: 0x32)
	             HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDRESS, 0x32 , 1, (uint8_t *) data_rec, 6, 10);
	  	 	    // Combine the two bytes for each axis
	  	 	    x = ((data_rec[1] << 8) | data_rec[0]);
	  	 	    y = ((data_rec[3] << 8) | data_rec[2]);
	  	 	    z = ((data_rec[5] << 8) | data_rec[4]);
	        // convert into g
	  	 	   xg = x * 0.0312;
	  	 	   yg = y * 0.0312;
	  	 	   zg = z * 0.0312;
        // calculate the magnitude
       	    a = sqrtf(powf(xg,2)+powf(yg,2)+powf(zg,2));


       // send data to queue
       xQueueSend(AccelQueueHandle,&a,10);
	   vTaskDelay(400/portTICK_RATE_MS);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_led */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led */
void led(void const * argument)
{
  /* USER CODE BEGIN led */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END led */
}

/* USER CODE BEGIN Header_ReadLocalion */
/**
* @brief Function implementing the ReadLocation thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadLocalion */
void ReadLocalion(void const * argument)
{
  /* USER CODE BEGIN ReadLocalion */
	  uint8_t location[100];
  /* Infinite loop */
  for(;;)
  {

	  NEO6_Task(&GpsState);

	  vTaskDelay(1000/portTICK_RATE_MS);
	  			if(NEO6_IsFix(&GpsState))
	  			{

	  			MessageLength = sprintf((char*)Message, "Latitude: %.2f %c\n\r", GpsState.Latitude, GpsState.LatitudeDirection);
	  			HAL_UART_Transmit(&huart2, Message, MessageLength, 100);

	  			MessageLength = sprintf((char*)Message, "Longitude: %06.2f %c\n\r", GpsState.Longitude, GpsState.LongitudeDirection);
	  			HAL_UART_Transmit(&huart2, Message, MessageLength, 100);
	  		    //sprintf(location, "Latitude: %.2f %c, Longitude: %06.2f %c", GpsState.Latitude, GpsState.LatitudeDirection, GpsState.Longitude, GpsState.LongitudeDirection);
	  		   //xQueueOverwrite( GPSQueueHandle, &location );
	  			}
	  	       else
	  				{
	  		   MessageLength = sprintf((char*)Message, "No Fix\n\r");
	  		   HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);
	  		   sprintf(location, "Latitude: 35.7 N, Longitude: 10.67 E");
	  		   xQueueOverwrite( GPSQueueHandle, &location );
	  			    }
	  			  	}


  /* USER CODE END ReadLocalion */
}

/* USER CODE BEGIN Header_DetectChute */
/**
* @brief Function implementing the DetectChuteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DetectChute */
void DetectChute(void const * argument)
{
  /* USER CODE BEGIN DetectChute */
	float acc;
	uint8_t ch[100],falldetected;
  /* Infinite loop */
  for(;;)
  {

	  		// read acceleration value
	          xQueueReceive(AccelQueueHandle, &acc,10);
	        	         // Wait for 1 second before checking acceleration again
	        		  	 // detect a free fall
	        		          if(acc<FALL_THRESHOLD_MIN)
	        		          {


	        		             vTaskDelay(700/portTICK_RATE_MS);
	        		             xQueueReceive(AccelQueueHandle, &acc,10);
	        		             //detect damage with the flour
	        		             if(acc>FALL_THRESHOLD_MAX){
	        		          	   vTaskDelay(2000/portTICK_RATE_MS);
	        		          	   int i=0;
	        		          	   falldetected = 1;
	        		          	   while (falldetected || i<10) {
	        		          		   sprintf(ch,"\t acceleration=%f \n", acc);

	        		          		    // Send the formatted data via UART
	        		          		    HAL_UART_Transmit(&huart2, (uint8_t *)ch, strlen(ch), 10);
	        		              	   // Wait for 1 second before checking acceleration again
	        		              	   vTaskDelay(1000/portTICK_RATE_MS);
	        		          		   xQueueReceive(AccelQueueHandle, &acc,10);

	        		          	        if (acc > 1.2 || acc < 0.8 ) {
	        		          	        // If acceleration value exceeds 1.2 or 0.8, exit loop
	        		          	        	falldetected=0;
	        		          	        }
	        		          	        i++;
	        		          	      }
	        		          	  if(falldetected==1)
	        		          	   {
	        		          		   sprintf(ch,"\t fall detected\n");
	        		         	       HAL_UART_Transmit(&huart2, (uint8_t *)ch, strlen(ch), 100);
	        		     	           xQueueSend(DetectChuteQueueHandle,&falldetected,10);
	        		          	   }
	        		             }
	        		          vTaskDelay(200/portTICK_RATE_MS);

	                 }

  }
  /* USER CODE END DetectChute */
}

/* USER CODE BEGIN Header_GSM */
/**
* @brief Function implementing the GSMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GSM */
void GSM(void const * argument)
{
  /* USER CODE BEGIN GSM */
    char msg[200];
	uint8_t buttonPressed,falldetected;
	char location[100];
  /* Infinite loop */
  for(;;)
  {

	  if( xQueueReceive( ButtonQueueHandle, &buttonPressed, 0 ))
	  		{
		  xQueueReceive(GPSQueueHandle,&location,0);
		  sprintf(msg,"Alerte ! Une situation critique s'est produite. Besoin d'assistance. Localisation: %s", location);
		  SendSms(msg);
	  		}

	  if( xQueueReceive( DetectChuteQueueHandle, &falldetected, 0 )  == pdPASS)
	  		{
		  xQueueReceive(GPSQueueHandle,&location,0);
		  sprintf(msg,"Urgence! détection de chute. Voici sa localisation: %s", location);
		  SendSms(msg);
	  		}
	  if(new_sms_received)
	  	  		{
		  if (strcmp(rx_buffer, "localisation"))
		  {
	  		  xQueueReceive(GPSQueueHandle,&location,0);
	  		  sprintf(msg,"Voici sa localisation: %s", location);
	  		  new_sms_received = 0; // Clear the flag
	  		  SendSms(msg);
		  }
	  	  		}

	  vTaskDelay(100/portTICK_RATE_MS);
  }
  /* USER CODE END GSM */
}

/* USER CODE BEGIN Header_AlertButton */
/**
* @brief Function implementing the AlertButtonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AlertButton */
void AlertButton(void const * argument)
{
  /* USER CODE BEGIN AlertButton */
	uint8_t buttonPressed=1;
  /* Infinite loop */
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)==0)
	  {
		  xQueueSend(ButtonQueueHandle, &buttonPressed, portMAX_DELAY);
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		  vTaskDelay(100);
	  }

  }
  /* USER CODE END AlertButton */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
