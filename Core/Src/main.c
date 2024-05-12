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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "algorithm.h"
#include "max_30102.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}	

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BRIGHTNESS 255
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value 
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid 指示SP02计算是否有效的指示器
int32_t n_heart_rate;   //heart rate value 心率值
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid显示心率计算是否有效的指示器
uint8_t uch_dummy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
	int i;
	int32_t n_brightness;
	float f_temp;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	maxim_max30102_reset(); //resets the MAX30102
	//read and clear status register
	maxim_max30102_read_reg(0,&uch_dummy);
	maxim_max30102_init();  //initializes the MAX30102
	printf("初始化完成 \r\n");   		
	n_brightness=0;
	un_min=0x3FFFF;	
	un_max=0;
	n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
	printf("进入循环 \r\n");
	//read the first 500 samples, and determine the signal range
	for(i=0;i<n_ir_buffer_length;i++)
	{
		// while(INT.read()==1);   //wait until the interrupt pin asserts
		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1){};   //wait until the interrupt pin asserts
		maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
		if(un_min>aun_red_buffer[i])
			un_min=aun_red_buffer[i];    //update signal min
		
		if(un_max<aun_red_buffer[i])
			un_max=aun_red_buffer[i];    //update signal max
		printf("red=");
		printf("%i", aun_red_buffer[i]);
		printf(", ir=");
		printf("%i \r\n", aun_ir_buffer[i]);
	}
	un_prev_data=aun_red_buffer[i];
	printf("循环完成 \r\n");
	//calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
	printf("开始主循环 \r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		i=0;
		
		un_min=0x3FFFF;
		
		un_max=0;

		//dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
		for(i=100;i<500;i++)
		{
		aun_red_buffer[i-100]=aun_red_buffer[i];
				
			aun_ir_buffer[i-100]=aun_ir_buffer[i];

			//update the signal min and max
			if(un_min>aun_red_buffer[i])
				
			un_min=aun_red_buffer[i];
			
			if(un_max<aun_red_buffer[i])
				un_max=aun_red_buffer[i];
		}

		//take 100 sets of samples before calculating the heart rate.
		for(i=400;i<500;i++)
		{
			un_prev_data=aun_red_buffer[i-1];
			//while(INT.read()==1);
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1){};   //wait until the interrupt pin asserts

			maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

			if(aun_red_buffer[i]>un_prev_data)//just to determine the brightness of LED according to the deviation of adjacent two AD data
			{
				f_temp=aun_red_buffer[i]-un_prev_data;
				
				f_temp/=(un_max-un_min);
				
				f_temp*=MAX_BRIGHTNESS;
				
				n_brightness-=(int)f_temp;
				
				if(n_brightness<0)
					n_brightness=0;
			}
			else
			{
				f_temp=un_prev_data-aun_red_buffer[i];
				
				f_temp/=(un_max-un_min);
				
				f_temp*=MAX_BRIGHTNESS;
				
				n_brightness+=(int)f_temp;
				
				if(n_brightness>MAX_BRIGHTNESS)
					n_brightness=MAX_BRIGHTNESS;
			}

			// pwmled.write(1-(float)n_brightness/256);//pwm control led brightness
//			if(n_brightness<120)	
//			{
//				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//			}
//			else
//			{
//				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//			}
			//send samples and calculation result to terminal program through UART
			printf("red=");
			
			printf("%i", aun_red_buffer[i]);
			
			printf(", ir=");
			
			printf("%i", aun_ir_buffer[i]);
			
			printf(", HR=%i, ", n_heart_rate); 
			
			printf("HRvalid=%i, ", ch_hr_valid);
			
			printf("SpO2=%i, ", n_sp02);
			
			printf("SPO2Valid=%i\n\r\n\r", ch_spo2_valid);
		}
		maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 

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
