/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "st7735\st7735.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
//112 e 144
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
  MX_SPI1_Init();
  ST7735_Init();
  /* USER CODE BEGIN 2 */
  int stoploop;

  int Plane_X, Plane_Y,Plane_pixelX, Plane_pixelY,Plane_speed,load_mass,Target_pixelX, Target_pixelY,location,cargo_pixelX, cargo_pixelY, vento = 0,direcao,cargo_Dropped = 0;
  char char_string[64], char_qstring[64], char_mass[] = "MASSA DA CARGA\0", char_height[] = "ALTURA DO        AVIAO\0", char_speed[] = "VELOCIDADE DO    AVIAO\0",char_wind[] = "FORCA DO VENTO\0", char_diretion[] = "DIRECAO DO       VENTO\0",char_direita[] = "DIREITA \0", char_esquerda[] = "ESQUERDA\0";
  char char_shape[] = "FORMATO DA CARGA\0", char_square[] = "QUADRADO\0", char_rec[] = "RETANGULO\0";

  InicializarSimulacao(&Plane_X, &Plane_Y,&Plane_pixelX, &Plane_pixelY,&Plane_speed,&load_mass,&Target_pixelX, &Target_pixelY, &location,&cargo_pixelX, &cargo_pixelY,&vento, &direcao, &stoploop, &cargo_Dropped);
  ST7735_FillScreen(BLACK);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while (stoploop < 3) {
		  ConfigurarParametros(char_qstring, char_string, &load_mass, &Plane_Y, &Plane_speed, &vento, &direcao, char_mass, char_height, char_speed, char_wind, char_esquerda, char_direita, char_diretion, &stoploop, &cargo_Dropped);
		  ST7735_FillScreen(BLACK);
		  Target_Spawn(Target_pixelX,Target_pixelY,&location);
		  Plane_Movement(Plane_Y, Plane_speed, Plane_X, Plane_pixelX, Plane_pixelY, location, cargo_pixelX, cargo_pixelY, vento, &cargo_Dropped);
		  stoploop++;
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7735_DC_Pin|ST7735_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ST7735_CS_Pin */
  GPIO_InitStruct.Pin = ST7735_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ST7735_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_DC_Pin ST7735_RES_Pin */
  GPIO_InitStruct.Pin = ST7735_DC_Pin|ST7735_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void InicializarSimulacao(int *Plane_X, int *Plane_Y,int *Plane_pixelX, int *Plane_pixelY,int *Plane_speed,int *load_mass,int *Target_pixelX, int *Target_pixelY,int *location,int *cargo_pixelX, int *cargo_pixelY,int *vento, int *direcao, int *stoploop, int *cargo_Dropped){
	*Plane_X = 1;
	*Plane_Y = 30;
	*Plane_pixelX = 15;
	*Plane_pixelY = 5;
	*Plane_speed = 3;
	*load_mass = 0;
	*Target_pixelX = 5;
	*Target_pixelY = 2;
	*location = 0;
	*cargo_pixelX = 2;
	*cargo_pixelY = 2;
	*vento = 20;
	*direcao = 1;
	*cargo_Dropped = 0;
	*stoploop = 0;

		ST7735_FillScreen(BLACK);
			ST7735_WriteString(20, 10, "SIMULADOR: ENTREGA", Font_7x10, WHITE, BLACK);
			ST7735_WriteString(20, 20, "DE SUPRIMENTOS", Font_7x10, WHITE, BLACK);
			HAL_Delay(2000);
		    ST7735_WriteString(20, 10, "                   ", Font_7x10, WHITE, BLACK);
		    ST7735_WriteString(20, 20, "              ", Font_7x10, WHITE, BLACK);
		    ST7735_WriteString(10, 10, "PRESSIONE PA10 PARA    CONTINUAR", Font_7x10, WHITE, BLACK);
			int buttonCheck = -1;
		    while (buttonCheck == -1) if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0) buttonCheck = 0;
}

void ConfigurarParametros (char *char_qstring, char *char_string, int* c_mass, int* p_height, int* p_speed, int* p_wind, int* p_windDirection, char* char_mass, char* char_height, char* char_speed, char* char_wind, char* char_esquerda, char* char_direita, char* char_diretion) {
		Option_Input(char_qstring, char_string,c_mass,char_mass);
		Option_Input(char_qstring, char_string,p_height, char_height);
		Option_Input(char_qstring, char_string,p_speed, char_speed);
		Option_Input(char_qstring, char_string,p_wind, char_wind);
		Bool_Option_Input(char_qstring, char_string,p_windDirection, char_diretion, char_esquerda, char_direita, p_wind);
}

void Option_Input(char *char_qstring, char *char_string, int *variable, char *char_x)
{
	int lm_button_check = -1, tempcheck_lm = -1;
	int maxValue = -1, minValue  = -1;
	while (tempcheck_lm == -1) {
		ST7735_FillScreen(BLACK);
		sprintf(char_qstring,"QUAL A %s?", char_x);
		ST7735_WriteString(5, 10, char_qstring, Font_7x10, WHITE, BLACK);
		ST7735_WriteString(10, 35, "-", Font_11x18, WHITE, BLACK);
		ST7735_WriteString(145, 40, "+", Font_11x18, WHITE, BLACK);
		ST7735_WriteString(48, 70, "CONFIRMAR", Font_7x10, WHITE, BLACK);
		sprintf(char_string, "%2d", *variable);
		ST7735_WriteString(65, 35, char_string, Font_11x18, WHITE, BLACK);

		while (lm_button_check == -1) {
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0) {
				(*variable)--;
				sprintf(char_string, "%2d", *variable);
				ST7735_WriteString(65, 35, char_string, Font_11x18, WHITE, BLACK);
				HAL_Delay(250);
			}
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0) {
				(*variable)++;
				sprintf(char_string, "%2d", *variable);
				ST7735_WriteString(65, 35, char_string, Font_11x18, WHITE, BLACK);
				HAL_Delay(250);
			}
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0) {
				lm_button_check = 0;
				tempcheck_lm = 0;
			}
		}
	}
}

void Bool_Option_Input(char *char_qstring, char *char_string,int *variable, char *char_x, char *option1, char *option2, int *vento)
{
	int lm_button_check = -1, tempcheck_lm = -1;
	int maxValue = 1, minValue = 0;
	while (tempcheck_lm == -1) {
		ST7735_FillScreen(BLACK);
		snprintf(char_qstring, 64, "QUAL A %s?", char_x);
		ST7735_WriteString(5, 10, char_qstring, Font_7x10, WHITE, BLACK);
		ST7735_WriteString(10, 35, "<-", Font_11x18, WHITE, BLACK);
		ST7735_WriteString(135, 35, "->", Font_11x18, WHITE, BLACK);
		ST7735_WriteString(48, 70, "CONFIRMAR", Font_7x10, WHITE, BLACK);

		memset(char_string, 0, 64);
		ST7735_WriteString(70, 35, "-", Font_7x10, WHITE, BLACK);
		while (lm_button_check == -1) {
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0 && *variable > minValue) {
				*variable = 0;
                memset(char_string, 0, 64);
                snprintf(char_string, 64, "%s", option1);
				ST7735_WriteString(50, 40, char_string, Font_7x10, BLACK, BLACK);
				ST7735_WriteString(50, 40, char_string, Font_7x10, WHITE, BLACK);
				HAL_Delay(250);
			}
			else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0 && *variable < maxValue) {
				*variable = 1;
                memset(char_string, 0, 64);
                snprintf(char_string, 64, "%s", option2);
                ST7735_WriteString(50, 40, char_string, Font_7x10, BLACK, BLACK);
                ST7735_WriteString(50, 40, char_string, Font_7x10, WHITE, BLACK);
				HAL_Delay(250);
			}

			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0) {
				if (*variable == 0) *vento = -*vento;
				lm_button_check = 0;
				tempcheck_lm = 0;
			}
		}
	}
}

void Plane_Movement(int Plane_Y, int Plane_speed, int Plane_X, int Plane_pixelX, int Plane_pixelY, int location, int cargo_pixelX, int cargo_pixelY, int vento, int *cargo_Dropped) {
	float dist_x, g = 9.8;
    float fall = sqrtf(2.0f * (75.0f - Plane_Y) / g);
    float drop_X = (float)location - ((Plane_speed + (float)vento/10.0f) * fall); // passar todas as linhas acima como referencia
	dist_x = Plane_speed * sqrtf(2.0f * (75.0f - Plane_Y));

	for (int i = 0; i < 180; i += (Plane_speed)) {
		if (*cargo_Dropped == 0) {
			ST7735_FillRectangle(Plane_X, Plane_Y, Plane_pixelX, Plane_pixelY, BLACK);
			Plane_X += (Plane_speed);
			ST7735_FillRectangle(Plane_X, Plane_Y, Plane_pixelX, Plane_pixelY, WHITE);
		}
		int x_position = i;
		if (Plane_X >= drop_X && *cargo_Dropped == 0) {
			ST7735_FillRectangle(Plane_X, Plane_Y, Plane_pixelX, Plane_pixelY, BLACK);
			ExecutarLancamento(cargo_pixelX, cargo_pixelY, Plane_X, Plane_Y, Plane_pixelX, Plane_pixelY, Plane_speed, vento, location);
			*cargo_Dropped = 1;
		}
	}
}

void Target_Spawn(int Target_pixelX, int Target_pixelY, int *location) {
		do {
			*location = rand() % 145;
		} while ((*location < 112));
		ST7735_FillRectangle(*location, 75, Target_pixelX , Target_pixelY, WHITE);
}

void ExecutarLancamento(int cargo_pixelX, int cargo_pixelY, int Plane_X, int Plane_Y, int Plane_pixelX, int Plane_pixelY, int Plane_speed, int vento, int location) {
    float x = Plane_X, y = Plane_Y;
    float g = 9.8f, t = 0;
    float fall = sqrtf(2.0f * (75.0f - Plane_Y) / g);
    float drop_X = (float)location - ((Plane_speed + (float)vento/10.0f) * fall);
    float vx = Plane_speed + (float)vento/10.0f;
    float vy = 0.0f;
    float time_step = 0.1f;
    int prev_x = x, prev_y = y;

    while (Plane_X < (int)drop_X || y < 75) {
    		ST7735_FillRectangle(Plane_X, Plane_Y, Plane_pixelX, Plane_pixelY, BLACK);
    		Plane_X += Plane_speed;
            ST7735_FillRectangle(Plane_X, Plane_Y, Plane_pixelX, Plane_pixelY, WHITE);

            ST7735_FillRectangle(prev_x, prev_y, cargo_pixelX, cargo_pixelY, BLACK);
            t += time_step;
            x += vx * time_step;
            y = Plane_Y + vy * t + 0.5f * g * t * t;
            vy += g * time_step;

            if (y > 75.0f) y = 75.0f;
            ST7735_FillRectangle((int)x, (int)y, cargo_pixelX, cargo_pixelY, RED);

            prev_x = (int)x;
            prev_y = (int)y;

    }ST7735_FillRectangle(prev_x, prev_y, cargo_pixelX, cargo_pixelY, RED);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
