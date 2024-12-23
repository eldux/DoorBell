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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE 10000   // 10 kHz sample rate
#define DAC_AMPLITUDE_DIV 4 //Divide volume by 4
#define DAC_RESOLUTION 2046/DAC_AMPLITUDE_DIV // 12-bit DAC maximum value
#define DAC_MIDPOINT DAC_RESOLUTION/2   // Midpoint value for 0 output
#define PI 3.14159265358979f

// Octave 3
#define NOTE_C3 130.81f
#define NOTE_CS3 138.59f // C sharp 3
#define NOTE_D3 146.83f
#define NOTE_DS3 155.56f // D sharp 3
#define NOTE_E3 164.81f
#define NOTE_F3 174.61f
#define NOTE_FS3 185.00f // F sharp 3
#define NOTE_G3 196.00f
#define NOTE_GS3 207.65f // G sharp 3
#define NOTE_A3 220.00f
#define NOTE_AS3 233.08f // A sharp 3
#define NOTE_B3 246.94f

// Octave 4
#define NOTE_C4 261.63f // Middle C
#define NOTE_CS4 277.18f // C sharp 4
#define NOTE_D4 293.66f
#define NOTE_DS4 311.13f // D sharp 4
#define NOTE_E4 329.63f
#define NOTE_F4 349.23f
#define NOTE_FS4 369.99f // F sharp 4
#define NOTE_G4 392.00f
#define NOTE_GS4 415.30f // G sharp 4
#define NOTE_A4 440.00f
#define NOTE_AS4 466.16f // A sharp 4
#define NOTE_B4 493.88f

// Octave 5
#define NOTE_C5 523.25f
#define NOTE_CS5 554.37f // C sharp 5
#define NOTE_D5 587.33f
#define NOTE_DS5 622.25f // D sharp 5
#define NOTE_E5 659.26f
#define NOTE_F5 698.46f
#define NOTE_FS5 739.99f // F sharp 5
#define NOTE_G5 783.99f
#define NOTE_GS5 830.61f // G sharp 5
#define NOTE_A5 880.00f
#define NOTE_AS5 932.33f // A sharp 5
#define NOTE_B5 987.77f

// Octave 6
#define NOTE_C6 1046.50f
#define NOTE_CS6 1108.73f // C sharp 6
#define NOTE_D6 1174.66f
#define NOTE_DS6 1244.51f // D sharp 6
#define NOTE_E6 1318.51f
#define NOTE_F6 1396.91f
#define NOTE_FS6 1479.98f // F sharp 6
#define NOTE_G6 1567.98f
#define NOTE_GS6 1661.22f // G sharp 6
#define NOTE_A6 1760.00f
#define NOTE_AS6 1864.66f // A sharp 6
#define NOTE_B6 1975.53f

// Octave 7
#define NOTE_C7 2093.00f
#define NOTE_CS7 2217.46f // C sharp 7
#define NOTE_D7 2349.32f
#define NOTE_DS7 2489.02f // D sharp 7
#define NOTE_E7 2637.02f
#define NOTE_F7 2793.83f
#define NOTE_FS7 2959.96f // F sharp 7
#define NOTE_G7 3135.96f
#define NOTE_GS7 3322.44f // G sharp 7
#define NOTE_A7 3520.00f
#define NOTE_AS7 3729.31f // A sharp 7
#define NOTE_B7 3951.07f

#define TEMPO 0.5 // Increase to speed up, decrease to slow down
// Rests
#define NOTE_REST 0 // Silence
// Durations in milliseconds
#define WHOLE_NOTE (1600 / TEMPO) // Whole note
#define HALF_NOTE  (800 / TEMPO)  // Half note
#define QUARTER_NOTE (400 / TEMPO) // Quarter note
#define EIGHTH_NOTE (200 / TEMPO) // Eighth note

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Define melody and chords
// Define a melody
typedef struct {
    float frequency;  // Frequency of the note (0 for pause)
    uint32_t durationMs; // Duration of the note
} Note;

// Melody array with pauses and various note durations
Note melody[] = {
    // Opening phrase
    {NOTE_E5, EIGHTH_NOTE}, {NOTE_E5, EIGHTH_NOTE}, {NOTE_REST, EIGHTH_NOTE}, {NOTE_E5, EIGHTH_NOTE},
    {NOTE_REST, EIGHTH_NOTE}, {NOTE_C5, EIGHTH_NOTE}, {NOTE_E5, EIGHTH_NOTE}, {NOTE_G5, QUARTER_NOTE},
    {NOTE_REST, QUARTER_NOTE}, {NOTE_G4, QUARTER_NOTE},

    // Second phrase
    {NOTE_C5, QUARTER_NOTE}, {NOTE_G4, QUARTER_NOTE}, {NOTE_E4, QUARTER_NOTE}, {NOTE_A4, EIGHTH_NOTE},
    {NOTE_B4, EIGHTH_NOTE}, {NOTE_A4, EIGHTH_NOTE}, {NOTE_GS4, EIGHTH_NOTE}, {NOTE_AS4, EIGHTH_NOTE},
    {NOTE_G4, EIGHTH_NOTE}, {NOTE_E5, EIGHTH_NOTE}, {NOTE_G5, EIGHTH_NOTE}, {NOTE_A5, HALF_NOTE},
    {NOTE_F5, EIGHTH_NOTE}, {NOTE_G5, EIGHTH_NOTE}, {NOTE_REST, EIGHTH_NOTE}, {NOTE_E5, QUARTER_NOTE},
    {NOTE_C5, EIGHTH_NOTE}, {NOTE_D5, EIGHTH_NOTE}, {NOTE_B4, EIGHTH_NOTE},

    // Third phrase
    {NOTE_C5, QUARTER_NOTE}, {NOTE_G4, QUARTER_NOTE}, {NOTE_E4, QUARTER_NOTE}, {NOTE_A4, EIGHTH_NOTE},
    {NOTE_B4, EIGHTH_NOTE}, {NOTE_A4, EIGHTH_NOTE}, {NOTE_GS4, EIGHTH_NOTE}, {NOTE_AS4, EIGHTH_NOTE},
    {NOTE_G4, EIGHTH_NOTE}, {NOTE_E5, EIGHTH_NOTE}, {NOTE_G5, EIGHTH_NOTE}, {NOTE_A5, HALF_NOTE},
    {NOTE_F5, EIGHTH_NOTE}, {NOTE_G5, EIGHTH_NOTE}, {NOTE_REST, EIGHTH_NOTE}, {NOTE_E5, QUARTER_NOTE},
    {NOTE_C5, EIGHTH_NOTE}, {NOTE_D5, EIGHTH_NOTE}, {NOTE_B4, EIGHTH_NOTE},

    // Ending phrase
    {NOTE_G5, QUARTER_NOTE}, {NOTE_FS5, QUARTER_NOTE}, {NOTE_F5, EIGHTH_NOTE}, {NOTE_DS5, EIGHTH_NOTE},
    {NOTE_E5, QUARTER_NOTE}, {NOTE_GS4, QUARTER_NOTE}, {NOTE_A4, QUARTER_NOTE}, {NOTE_C5, EIGHTH_NOTE},
    {NOTE_A4, EIGHTH_NOTE}, {NOTE_C5, QUARTER_NOTE}, {NOTE_D5, QUARTER_NOTE},
    {NOTE_G5, QUARTER_NOTE}, {NOTE_FS5, QUARTER_NOTE}, {NOTE_F5, EIGHTH_NOTE}, {NOTE_DS5, EIGHTH_NOTE},
    {NOTE_E5, QUARTER_NOTE}, {NOTE_C5, QUARTER_NOTE}, {NOTE_E5, QUARTER_NOTE}, {NOTE_A5, QUARTER_NOTE},
    {NOTE_REST, QUARTER_NOTE}, {NOTE_A5, EIGHTH_NOTE}, {NOTE_AS5, EIGHTH_NOTE}, {NOTE_A5, EIGHTH_NOTE},
    {NOTE_G5, EIGHTH_NOTE}, {NOTE_E5, QUARTER_NOTE}, {NOTE_G5, QUARTER_NOTE}, {NOTE_A5, QUARTER_NOTE},
    {NOTE_F5, EIGHTH_NOTE}, {NOTE_G5, EIGHTH_NOTE}, {NOTE_REST, EIGHTH_NOTE},
};

#define MAX_SAMPLES 2000 // Buffer size for one note
uint16_t soundBuffer1[MAX_SAMPLES]; // Buffer 1
uint16_t soundBuffer2[MAX_SAMPLES]; // Buffer 2
uint16_t* currentBuffer = soundBuffer1; // Active buffer
uint16_t* nextBuffer = soundBuffer2;    // Buffer for preloading
uint32_t currentMelodyIndex = 0;
float sinePhase = 0; // Global sine phase
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void PlayNote(Note note);
void GenerateNoteWaveform(float frequency, uint32_t durationMs, uint16_t* buffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function to generate the waveform for a single note or pause
void GenerateNoteWaveform(float frequency, uint32_t durationMs, uint16_t* buffer) {
    uint32_t numSamples = (SAMPLE_RATE * durationMs) / 1000;
    if (numSamples > MAX_SAMPLES) numSamples = MAX_SAMPLES;

    uint32_t fadeInSamples = numSamples * 5 / 100;  // 5% fade-in
    uint32_t fadeOutSamples = numSamples * 40 / 100; // 40% fade-out
    uint32_t fadeOutStart = numSamples - fadeOutSamples;

    for (uint32_t i = 0; i < numSamples; i++) {
        float amplitude = 1.0;

        // Fade-in logic
        if (i < fadeInSamples) {
            amplitude = (float)i / fadeInSamples;
        }
        // Fade-out logic
        else if (i >= fadeOutStart) {
            amplitude = (float)(numSamples - i) / fadeOutSamples;
        }

        // Sine wave or silence
        if (frequency > 0) {
            buffer[i] = (uint16_t)(DAC_MIDPOINT + (DAC_RESOLUTION / 2) * amplitude * sin(sinePhase));
            sinePhase += 2 * PI * frequency / SAMPLE_RATE;
            if (sinePhase >= 2 * PI) sinePhase -= 2 * PI; // Keep phase continuous
        } else {
            buffer[i] = DAC_MIDPOINT; // Center value for silence
        }
    }
}

// Start playing a note
void PlayNote(Note currentNote) {
    // Start DMA transfer for the current buffer
    uint32_t numSamples = (SAMPLE_RATE * currentNote.durationMs) / 1000;
    if (numSamples > MAX_SAMPLES) numSamples = MAX_SAMPLES;

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)currentBuffer, numSamples, DAC_ALIGN_12B_R);

    // Generate waveform for the next note in the background
    Note nextNote = melody[(currentMelodyIndex + 1) % (sizeof(melody) / sizeof(Note))];
    GenerateNoteWaveform(nextNote.frequency, nextNote.durationMs, nextBuffer);
}

// DMA completion callback
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    // Advance to the next note
    currentMelodyIndex++;
    if (currentMelodyIndex >= sizeof(melody) / sizeof(Note)) {
        currentMelodyIndex = 0; // Loop melody
    }

    // Swap buffers
    uint16_t* temp = currentBuffer;
    currentBuffer = nextBuffer;
    nextBuffer = temp;

    // Play the current note and prepare the next note
    PlayNote(melody[currentMelodyIndex]);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // Start playing the first note
  PlayNote(melody[currentMelodyIndex]);

  // Start the timer to trigger note changes
  // Start the timer to trigger the DAC
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(100);
	  if (!HAL_GPIO_ReadPin(MUSIC_SEL_BTN_GPIO_Port, MUSIC_SEL_BTN_Pin))
	  {
		  HAL_GPIO_WritePin(AMP_ON_GPIO_Port, AMP_ON_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(AMP_ON_GPIO_Port, AMP_ON_Pin, GPIO_PIN_RESET);
	  }
	  /*
	  	  HAL_GPIO_WritePin(LED_CTRL_GPIO_Port, LED_CTRL_Pin, GPIO_PIN_RESET);
	  	  HAL_Delay(900);
	  	  HAL_GPIO_WritePin(LED_CTRL_GPIO_Port, LED_CTRL_Pin, GPIO_PIN_SET);
	  	  HAL_Delay(100);
	  	  */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_CTRL_GPIO_Port, LED_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMP_ON_GPIO_Port, AMP_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DOOR_BTN_Pin BRIGHTNESS_BTN_Pin VOL_UP_BTN_Pin */
  GPIO_InitStruct.Pin = DOOR_BTN_Pin|BRIGHTNESS_BTN_Pin|VOL_UP_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VOL_DOWN_BTN_Pin MUSIC_SEL_BTN_Pin */
  GPIO_InitStruct.Pin = VOL_DOWN_BTN_Pin|MUSIC_SEL_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_CTRL_Pin */
  GPIO_InitStruct.Pin = LED_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AMP_ON_Pin */
  GPIO_InitStruct.Pin = AMP_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMP_ON_GPIO_Port, &GPIO_InitStruct);

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
