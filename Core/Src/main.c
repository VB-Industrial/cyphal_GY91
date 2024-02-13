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
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//I2C_HandleTypeDef hi2c4;
extern I2C_HandleTypeDef hi2c4;
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
  MX_FDCAN1_Init();
  MX_I2C4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  cyphal_can_starter(&hfdcan1);
  setup_cyphal(&hfdcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint8_t msg[10];
  int i = 0;

  //uint16_t addr9250=0xD0; //0x68<<1
  uint8_t bufAG[14] = {0};
  HAL_StatusTypeDef rv;

  uint16_t sAx,sAy,sAz,sGx,sGy,sGz; //axel-gyro components
  int16_t xx;

  uint32_t last_hbeat = HAL_GetTick();
  float pos = JOINT_N;
  float vel = JOINT_N;
  float eff = JOINT_N;

  float x,y,z,vx,vy,vz = 0;


  MPU6050_Init(&hi2c4);

  while (1)
  {

//	  	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
//
//	  	rv = HAL_I2C_IsDeviceReady(&hi2c4, 0xD0, 2, 5);
		rv = HAL_I2C_Mem_Read_IT(&hi2c4, 0xD0, 0x3B, 1, &bufAG[0], 14);

	   xx = ( bufAG[0] << 6) + (bufAG[1]>>2) + 8192; if (xx > 16383) {xx -= 16383;}; sAx=xx;
	   xx = ( bufAG[2] << 6) + (bufAG[3]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sAy=xx;
	   xx = ( bufAG[4] << 6) + (bufAG[5]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sAz=xx;
	   xx = ( bufAG[8] << 6) + (bufAG[9]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sGx=xx;
	   xx = ( bufAG[10] << 6) + (bufAG[11]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sGy=xx;
	   xx = ( bufAG[12] << 6) + (bufAG[13]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sGz=xx;

	   x = (((float)(8192 - sAx))/ 163834) * 2 * (float)M_PI;
	   y = (((float)(8192 - sAy))/ 16384) * 2 * (float)M_PI;
	   z = (((float)(8192 - sAz))/ 16384) * 2 * (float)M_PI;
	   vx = (((float)(8192 - sGx))/ 16384) * 2 * (float)M_PI;
	   vy = (((float)(8192 - sGy))/ 16384) * 2 * (float)M_PI;
	   vz = (((float)(8192 - sGz))/ 16384) * 2 * (float)M_PI;


	   send_IMU(&x, &y, &z, &vx, &vy, &vz);

//
//	   if (rv == HAL_OK)
//	   {
//		   i = 111;
//	   }
//	   else
//	   {
//		   i = 000;
//	   }
//
//	  sprintf(msg,"%f \n\0", x);
//	  HAL_UART_Transmit_IT(&huart2, msg, sizeof(msg));

      cyphal_loop();
      HAL_Delay(50);



//      uint32_t now = HAL_GetTick();
//      if ( (now - last_hbeat) >= 100) {
//          last_hbeat = now;
//          heartbeat();
//          pos += 0.01;
//          vel += 0.01;
//          eff += 0.01;
//          send_JS(&pos, &vel, &eff);
//          if(pos > 1.5)
//          {
//        	  pos = 0.0;
//        	  vel = 0.0;
//        	  eff = 0.0;
//          }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
	HAL_StatusTypeDef rv;
    uint8_t check = 0;
    uint8_t Data = 0;

    // check device ID WHO_AM_I

    rv = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1);
        return 0;
    }
    return 1;
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
