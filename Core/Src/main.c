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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <string.h>
#include <MLX90640_I2C_Driver.h>
#include <MLX90640_API.h>
#include "myiic.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t numRows;   /**< number of rows in the data table. */
	uint16_t numCols;   /**< number of columns in the data table. */
	float *pData;   /**< points to the data table. */
} bilinear_interp_instance_f32;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS16HZ 
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
paramsMLX90640 mlx90640;
static uint16_t eeMLX90640[832];    //mlx90640 EEPROM
static uint16_t mlx90640Frame[834]; //mlx90640 原始数据
static float mlx90640To[768];       //mlx90640 温度数据
static float data_To[src_x*src_y]={0};
static float tr;
int status;
uint16_t lcd_buffer[Buffer_cols*Buffer_rows]={0};
float data_To_max=0;
float data_To_min=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t GrayToPseColor(uint8_t grayValue)  //灰度-伪彩色变�?
{
	uint8_t colorR,colorG,colorB; 
	if( (grayValue>=0) && (grayValue<=63) )  
	{
		colorR=0;
		colorG=0;
		colorB=grayValue/2;
	}
	else if( (grayValue>=64) && (grayValue<=127) )  
	{
		colorR=0;
		colorG=grayValue-64;
		colorB=(127-grayValue)/2;
	}
	else if( (grayValue>=128) && (grayValue<=191) )  
	{
		colorR=(grayValue-128)/2;
		colorG=63;
		colorB=0;
	}
	else if( (grayValue>=192) && (grayValue<=255) )  
	{
		colorR=31;
		colorG=255-grayValue;
		colorB=0;
	}
  return (colorR<<11)|(colorG<<5)|colorB;
}

float bilinear_interp(float pData[src_x*src_y],float X,float Y)
{
	float out;
	float f00, f01, f10, f11;
	uint16_t xIndex, yIndex;
	float xdiff, ydiff,_1_xdiff,_1_ydiff;

	xIndex = (uint16_t) X;
	yIndex = (uint16_t) Y;

//	if(xIndex==src_x-1)
//	{
////		return 0;
//		xIndex--;
//	}
//	if(yIndex==src_y-1)
//	{
////		return 0;
//		yIndex--;
//	}
	f00 = pData[yIndex*src_x+xIndex];
	f01 = pData[(yIndex+1)*src_x+xIndex];
	f10 = pData[yIndex*src_x+xIndex+1];
	f11 = pData[(yIndex+1)*src_x+xIndex+1];
	xdiff = (X-xIndex);
	ydiff = (Y-yIndex);
	_1_xdiff = 1-X+xIndex;
	_1_ydiff = 1-Y+yIndex;
	out = f00*_1_xdiff*_1_ydiff+f10*xdiff*_1_ydiff+f01*_1_xdiff*ydiff+f11*xdiff*ydiff;
	return out;
}
	
void Temp2Gray_all(float temp[src_x*src_y])
{
	float dx = (float)(src_x-1)/dst_x;
	float dy = (float)(src_y-1)/dst_y;
	uint32_t n_gray=0;
	float gray=0;
	for (int y=0;y<dst_y;y++)
	{
		for(int x=0;x<dst_x;x++)
		{
			gray=bilinear_interp(temp,x*dx,y*dy);
			gray=255.0f*(gray-data_To_min)/(data_To_max-data_To_min);
			lcd_buffer[n_gray++] = GrayToPseColor(gray);
		}
	}
	Lcd_Write_16(0,0,dst_x,dst_y,lcd_buffer);
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim5);
	I2C_Init();
	HAL_Delay(100);
	
	status = MLX90640_SetResolution(MLX90640_ADDR,0x03);//设置16bit分辨�?
  status = MLX90640_SetRefreshRate(MLX90640_ADDR,RefreshRate);//设置为刷新率16Hz
	status = MLX90640_SetChessMode(MLX90640_ADDR);//棋盘模式
	status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);//读取 EEPROM
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);//�? EEPROM 计算得到计算参数

	int mode = MLX90640_GetCurMode(MLX90640_ADDR);
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		status = MLX90640_GetFrameData(MLX90640_ADDR, mlx90640Frame);
		if(status == 0||status == 1)
		{
			MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95, tr, mlx90640To);
		}
		if(status==1)
		{
			data_To_max=5;
			data_To_min=20;
			tr = MLX90640_GetTa(mlx90640Frame, &mlx90640)-TA_SHIFT;
			MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx90640To, mode, &mlx90640);
			MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx90640To, mode, &mlx90640);
			uint16_t data_n=0;
			for(uint8_t y=0;y<src_y;y++)
			{
				for(uint8_t x=0;x<src_x;x++)
				{
					uint16_t n_src=767-32*x-y;
					data_To[data_n]=*(mlx90640To+n_src);
					if(data_To[data_n]>data_To_max)	
					{
						data_To_max=data_To[data_n];
					}
					if(data_To[data_n]<data_To_min)
					{
						data_To_min=data_To[data_n];
					}
					data_n++;
				}
			}
			Temp2Gray_all(data_To);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
