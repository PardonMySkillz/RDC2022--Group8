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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "camera/camera.h"
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
// #include "legacy/camera.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void medianFilter(uint16_t width, uint16_t height, uint16_t* originaldataPtr, uint16_t* processed_dataPtr) {
    uint8_t window[9];

    for (uint16_t x=1; x<width-1; x++) {
        for (uint16_t y=1; y<height-1; y++) {
            uint8_t k = 0;
            for (int u=x-1; u<=x+1;u++) {
                for (int v=y-1; v<=y+1; v++) {
                    window[k++] = *(originaldataPtr + width*v+u);
                }
            }
            for (uint8_t i=0; i<5; i++) {
                for (uint8_t j=i+1; j<9; j++) {
                    if (window[j] < window[i]) {
                        uint8_t temp = window[i];
                        window[i] = window[j];
                        window[j] = temp;
                    }
                }
            }
            *(processed_dataPtr  + width*y+x) = window[4];
        }
    }
}

void sobelOperation(uint16_t width, uint16_t height, uint8_t* originalPTR, uint8_t* processed_dataPtr) {
    int16_t window[9];
    int16_t gx[] = {1,0,-1,2,0,-2,1,0,-1};
    int16_t gy[] = {1,2,1,0,0,0,-1,-2,-1};

    for (uint16_t x=1; x<width-1; x++) {
        for (uint16_t y=1; y<height-1; y++) {
            uint8_t k =0;
            for (int u=x-1; u <=x+1; u++) {
                for (int v=y-1; v<=y+1; v++) {
                    window[k++] = *(originalPTR + width*v+u);
                }
            }
            //do sobel operation gx^2 + gy^2
            int8_t x_overall =0;
            int8_t y_overall = 0;
            for (uint16_t i=0; i<9; i++) {
                x_overall += window[i]*gx[i];
                y_overall += window[i]*gy[i];
            }
            int16_t sobel_pixel = sqrt(pow(x_overall,2) + pow(y_overall,2));
            *(processed_dataPtr  +width*y+x) = sobel_pixel;
        }
    }
}

void overallImgProcessor(uint16_t width, uint16_t height, uint16_t* originaldataPtr, uint8_t* processed_dataPtr) {
	image_processing_to_gray_scale(width, height, originaldataPtr, processed_dataPtr);
	medianFilter(width, height, originaldataPtr, processed_dataPtr);
	sobelOperation( width, height, originaldataPtr, processed_dataPtr);
}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
//  MX_SPI1_Init();
  MX_USART1_UART_Init();
//  MX_I2C2_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  led_off(LED1);
  led_off(LED2);
  led_off(LED3);
  led_off(LED4);
  tft_init(PIN_ON_TOP, BLACK, WHITE, YELLOW, DARK_GREEN);

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
  	cam_set_window(0, 0, QQVGA_120x160);
//  	cam_set_framesize(QQVGA_120x160);
//  	cam_set_framerate(CAM_75FPS);
  	cam_set_colormode(CAM_GRAYSCALE);
  //	cam_set_lightmode(CAM_LIGHT_AUTO);
  //	cam_set_effect(CAM_FX_BW);
  //	cam_set_brightness(0);
  //	cam_set_saturation(0);
  //	cam_set_contrast(0);


  	//init the pwm pins//
  //	TIM10 ->ARR = 839;
  //	TIM11 ->ARR = 839;
  //	TIM5 ->ARR = 839;
  //
  //	TIM10 ->PSC = 9;
  //	TIM11 ->PSC = 9;
  //	TIM5 ->PSC = 9;

#define IMG_WIDTH 120
#define IMG_HEIGHT 160

  	uint16_t image[IMG_HEIGHT*IMG_WIDTH] = {0};
//  	uint16_t img_data[IMG_HEIGHT*IMG_WIDTH] = {0};
  	uint16_t processed[IMG_HEIGHT*IMG_WIDTH] = {0};

  	while (1)
  	{
//  		gpio_reset(LED1);
  		//tft_prints(0,0,"Hello World!");
  		//tft_update(10);
  		if (cam_is_frame_ready())
  		{
  			cam_get_rgb565(image);
  			cam_rgb2printable(image, processed);
  			tft_print_image(processed,0,0,120,160);
  		}
  		//overallImgProcessor(IMG_WIDTH, IMG_HEIGHT, img_data_ptr, processed_image_ptr);
  		static uint32_t last_ticks = 0;
  		if (HAL_GetTick() - last_ticks > 100) {
  			last_ticks = HAL_GetTick();
  			led_toggle(LED1);
  		}

  		//tft_print_image(processed_image_ptr, 0, 0, 120, 160);

  	}
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
