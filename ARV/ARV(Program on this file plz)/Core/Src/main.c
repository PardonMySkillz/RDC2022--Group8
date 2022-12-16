/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include <stdlib.h>
#include "camera/camera.h"
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void blackFilter(uint16_t width, uint16_t height, uint16_t* originalPTR, uint16_t* processedPTR)
{
	for (uint16_t x=0; x<width; x++)
	{
		for (uint16_t y =0; y<height; y++)
		{
			uint16_t pixel = *(originalPTR + width*y + x) & 0x001F;
			if (pixel > 11)
			{
				*(processedPTR + width*y + x) = *(processedPTR + width*y + x) | 0xFFFF;
			}
			else
			{
				*(processedPTR + width*y + x) = *(processedPTR + width*y + x) & 0x0000;
			}
		}
	}
}

void sobelFilter(uint16_t width, uint16_t height, uint16_t* originalPTR, uint16_t* processed_dataPtr){
	int16_t window[9]; //set an array to store values of 3x3 matrix
    int16_t gx[] = {1,0,-1,2,0,-2,1,0,-1};
    int16_t gy[] = {1,2,1,0,0,0,-1,-2,-1};

    for (uint16_t x=1; x<width-1; x++) //outer edge of image will be ignored
    {
        for (uint16_t y=1; y<height-1; y++) //outer edge of image will be ignored
        {
        	//fill the 3x3 window
            uint8_t k =0;
            for (int u=x-1; u <=x+1; u++) //horizontal
            {
                for (int v=y-1; v<=y+1; v++) //vertical
                {
                    window[k++] = *(originalPTR + width * v + u);
                }
            }
            //Pass through SobelFilter
            int8_t sumX = 0;
            int8_t sumY = 0;

            for (uint16_t i = 0; i < 9; i++)
            {
            	uint16_t blue = (window[i] & 0x001F);
                sumX += blue*gx[i];
                sumY += blue*gy[i];
            }
            uint16_t magnitudeBlue = sqrt(pow(sumX, 2) + pow(sumY, 2));
            if (magnitudeBlue < 12)
            {
            	magnitudeBlue = 0;
            }
            else
            {
            	magnitudeBlue = 31;
            }

            uint16_t temporary =  ((((magnitudeBlue * 2)+1) | (magnitudeBlue << 6)) << 5) | magnitudeBlue;
            //uint16_t temporary = ((magnitudeBlue | temporary) << 6); // red
            //temporary = (((magnitudeBlue * 2)+1) | temporary) << 5; // green
            //temporary = magnitudeBlue | temporary; // blue
            *(processed_dataPtr + width * y + x) = temporary;
        }
    }
}

void pwm_init(void) {
	// init the pwm prescaler value and auto-reload value and start the pwm
	/* Your code start here */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); //servoMotor
	TIM5->ARR = 39999;
	TIM5->PSC = 41;
	TIM5->CCR1 = 2999;
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); //leftMotor
	TIM10->ARR = 839;
	TIM10->PSC = 9;
	TIM10->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); //rightMotor
	TIM11->ARR = 839;
	TIM11->PSC = 9;
	TIM11->CCR1 = 0;
}
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void forward()
{
	gpio_set(IN1);
	gpio_reset(IN2);
	gpio_reset(IN3);
	gpio_set(IN4);
}

void steeringAlgorithm(uint16_t* image, uint16_t width, uint16_t height)
{
	uint16_t left = 0;
	uint16_t right = 0;
	for (uint16_t y = 0; y<height/4-1; y++)
	{
		for (uint16_t x = 0; x<width/2-1; x++) // Calculate Left Rectangle
		{
			if (*(image + width*y + x) == 0xFFFF)
			{
				right++;
			}
		}
		for (uint16_t x = width/2-1;x<width-1;x++) // Calculate Right Rectangle
		{
			if (*(image + width*y + x) == 0xFFFF)
			{
				left++;
			}
		}
	}

	if (abs(left-right) >= 250) //find difference of number of pixels between left and right
	{
		if (left > right)
		{
			TIM5->CCR1 = 1680;
		}
		else
		{
			TIM5->CCR1 = 4200;
		}
	}
	else
	{
		TIM5->CCR1 = 3200;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  //MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  led_off(LED1);
  led_off(LED2);
  led_off(LED3);
  led_off(LED4);
  tft_init(PIN_ON_BOTTOM, BLACK, WHITE, YELLOW, DARK_GREEN);

  camera_GPIO_init();
  	tft_prints(0, 0, "Initing camera");
  	tft_update(0);
  	if (camera_init() == CAM_NOT_INITED || camera_init() == CAM_INIT_ERROR) {
  		tft_prints(0, 0, "No OV7725 module");
  	}
  	else {
  		tft_prints(0, 0, "Inited");
  		cam_set_state(CAM_CAPTURING);
  	}
  	tft_update(0);
  	cam_set_window(70, 0, QQVGA_120x160);
  	cam_set_zoomscale(0);
//  	cam_set_framesize(QQVGA_120x160);
  	cam_set_framerate(CAM_75FPS);
  	cam_set_colormode(CAM_GRAYSCALE);
//  	cam_set_lightmode(CAM_LIGHT_AUTO);
  //	cam_set_effect(CAM_FX_BW);
  //	cam_set_brightness(0);
  //
  //cam_set_saturation(0);
  //	cam_set_contrast(0);
  //	init the pwm pins//
  	pwm_init();

#define IMG_WIDTH 120
#define IMG_HEIGHT 160
  	const int WIDTH = cam_sizes[cam_get_framesize()].width, HEIGHT = cam_sizes[cam_get_framesize()].height;

  	uint16_t image[HEIGHT*WIDTH];
//  	uint16_t img_data[IMG_HEIGHT*IMG_WIDTH] = {0};
  	uint16_t processed[HEIGHT*WIDTH];

  	//uint16_t printable[IMG_HEIGHT*IMG_WIDTH] = {0};

  	while (1)
  	{
  		static uint16_t last_ticks = 0;
  		if (cam_is_frame_ready())
  		{
  			//Get image from camera
  			cam_get_rgb565(image);
  			//Commence SobelOperation:
  			if (HAL_GetTick() - last_ticks >= 100)
			{
  				blackFilter(WIDTH, HEIGHT, image, processed);
				led_toggle(LED1);
				last_ticks = HAL_GetTick();
			}
  			steeringAlgorithm(processed, WIDTH, HEIGHT);
  			tft_update(0);
  			cam_rgb2printable(processed, image);
  			//Print Image
  			tft_print_image(image,0,0,WIDTH,HEIGHT);
  		}
  		//Motors
		TIM10->CCR1 = 620;
		TIM11->CCR1 = 620;
		//forward();
  	}
  /* USER CODE END 2 */
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
	while (1) {}
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
