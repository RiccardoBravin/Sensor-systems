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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_SIZE 100
#define N_CHARS 38 //17 because we added X to the end when we receive something else than the expected chars
const uint8_t led_mat[N_CHARS][5][2] = {
		//A
		{{31, 16},
		{36, 8},
		{68, 4},
		{36, 2},
		{31, 1}},
		//B
		{{127, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{54, 1}},
		//C
		{{62, 16},
		{65, 8},
		{65, 4},
		{65, 2},
		{34, 1}},
		//D
		{{127, 16},
		{65, 8},
		{65, 4},
		{65, 2},
		{62, 1}},
		//E
		{{127, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{73, 1}},
		//F
		{{127, 16},
		{72, 8},
		{72, 4},
		{72, 2},
		{72, 1}},
		//G
		{{127, 16},
		{65, 8},
		{73, 4},
		{73, 2},
		{39, 1}},
		//H
		{{127, 16},
		{8, 8},
		{8, 4},
		{8, 2},
		{127, 1}},
		//I
		{{0, 16},
		{65, 8},
		{127, 4},
		{65, 2},
		{0, 1}},
		//J
		{{0, 16},
		{6, 8},
		{65, 4},
		{126, 2},
		{64, 1}},
		//K
		{{127, 16},
		{8, 8},
		{20, 4},
		{34, 2},
		{65, 1}},
		//L
		{{0, 16},
		{127, 8},
		{1, 4},
		{1, 2},
		{0, 1}},
		//M
		{{127, 16},
		{64, 8},
		{48, 4},
		{64, 2},
		{127, 1}},
		//N
		{{127, 16},
		{48, 8},
		{8, 4},
		{6, 2},
		{127, 1}},
		//O
		{{62, 16},
		{65, 8},
		{65, 4},
		{65, 2},
		{62, 1}},
		//P
		{{127, 16},
		{72, 8},
		{72, 4},
		{72, 2},
		{48, 1}},
		//Q
		{{62, 16},
		{65, 8},
		{69, 4},
		{62, 2},
		{1, 1}},
		//R
		{{127, 16},
		{72, 8},
		{76, 4},
		{74, 2},
		{49, 1}},
		//S
		{{49, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{70, 1}},
		//T
		{{64, 16},
		{64, 8},
		{127, 4},
		{64, 2},
		{64, 1}},
		//U
		{{126, 16},
		{1, 8},
		{1, 4},
		{1, 2},
		{126, 1}},
		//V
		{{124, 16},
		{2, 8},
		{1, 4},
		{2, 2},
		{124, 1}},
		//W
		{{126, 16},
		{1, 8},
		{6, 4},
		{1, 2},
		{126, 1}},
		//X
		{{99, 16},
		{20, 8},
		{8, 4},
		{20, 2},
		{99, 1}},
		//Y
		{{64, 16},
		{32, 8},
		{31, 4},
		{32, 2},
		{64, 1}},
		//Z
		{{67, 16},
		{69, 8},
		{73, 4},
		{81, 2},
		{97, 1}},
		//0
		{{62, 16},
		{113, 8},
		{73, 4},
		{71, 2},
		{62, 1}},
		//1
		{{16, 16},
		{33, 8},
		{127, 4},
		{1, 2},
		{0, 1}},
		//2
		{{33, 16},
		{67, 8},
		{69, 4},
		{73, 2},
		{49, 1}},
		//3
		{{34, 16},
		{65, 8},
		{73, 4},
		{73, 2},
		{54, 1}},
		//4
		{{120, 16},
		{8, 8},
		{8, 4},
		{8, 2},
		{127, 1}},
		//5
		{{114, 16},
		{81, 8},
		{81, 4},
		{81, 2},
		{78, 1}},
		//6
		{{62, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{38, 1}},
		//7
		{{64, 16},
		{64, 8},
		{79, 4},
		{80, 2},
		{96, 1}},
		//8
		{{54, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{54, 1}},
		//9
		{{50, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{62, 1}},
		//dot
		{{0, 16},
		{0, 8},
		{1, 4},
		{0, 2},
		{0, 1}},
		//space
		{{0, 16},
		{0, 8},
		{0, 4},
		{0, 2},
		{0, 1}}
};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct Queue {
	uint8_t items[QUEUE_SIZE];
	int front;
	int rear;
	int size;
} rx_queue;


uint8_t rx_data = 0;
int rx_letter_idx = 16;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void init_queue(struct Queue* q);
int isFull(struct Queue *queue);
int isEmpty(struct Queue *queue);
int enqueue(struct Queue *queue, uint8_t item);
uint8_t dequeue(struct Queue *queue);
uint8_t* front(struct Queue *queue);
uint8_t char_to_ind(uint8_t c);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//management of IR reception
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(isEmpty(&rx_queue)) HAL_TIM_Base_Start_IT(&htim11); //start the timer to dequeue received letters

	enqueue(&rx_queue, rx_data); //add to queue received data

	HAL_UART_Receive_IT(&huart1, &rx_data, 1); //start new reception
}



//management of led matrix
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim == &htim10){
		static int idx = 0;
		HAL_SPI_Transmit_DMA(&hspi1, led_mat[rx_letter_idx][idx], 2);
		idx = (idx + 1) % 5;
	}else if(htim == &htim11){//change letters each half second
		uint8_t to_display = dequeue(&rx_queue);
		rx_letter_idx = char_to_ind(to_display);

		if(isEmpty(&rx_queue)) //if the queue is empty we do not have to do this nomore
			HAL_TIM_Base_Stop_IT(&htim11);
	}
}
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  init_queue(&rx_queue);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  HAL_TIM_Base_Start_IT(&htim10);


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 39;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8400-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 5000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_RX;
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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void init_queue(struct Queue* q){
	q->front = 0;
	q->rear = QUEUE_SIZE - 1;
	q->size = 0;
}
int isFull(struct Queue *queue) {
	return queue->size == QUEUE_SIZE;
}

int isEmpty(struct Queue *queue) {
	return (queue->size == 0);
}

int enqueue(struct Queue *queue, uint8_t item) {
	if (isFull(queue))
		return 0;
	queue->rear = (queue->rear + 1) % QUEUE_SIZE;
	queue->items[queue->rear] = item;
	queue->size = queue->size + 1;
	return 1;
}

uint8_t dequeue(struct Queue *queue) {
	if (isEmpty(queue))
		Error_Handler(); //if dequeue with empty queue return error!

	uint8_t item = queue->items[queue->front];
	queue->front = (queue->front + 1) % QUEUE_SIZE;
	queue->size = queue->size - 1;
	return item;
}

uint8_t* front(struct Queue* queue){
    if (isEmpty(queue)) Error_Handler();
    return &(queue->items[queue->front]);
}

uint8_t char_to_ind(uint8_t c){
	if(c >= 'A' && c <='Z'){
		return c - 'A';
	}else if(c >= 'a' && c <='z'){
		return c - 'a';
	}else if(c >= '0' && c <= '9'){
		return c - '0' + 26; //6 because the first 26 mat are letters
	}else{
		return N_CHARS - 1;
	}

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
