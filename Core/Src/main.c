/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <joint_mapper.h>
#include "stepper/stepper_hw.h"
//#include "axis.h"
//#include "sync_motion.h"
#include "system_config.h"
#include "math_utils.h"
#include "app.h"
#include "stepper/homing_control.h"
#include "sts_servo/sts_bus.h"
#include "sts_servo/sts3215.h"

#include "pc_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


//typedef struct{
//    float q;         // 現在角度 [rad]
//    float dq;        // 現在角�????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��度 [rad/s]
//    float q_target;  // 目標角度 [rad]
//    float v_max;     // ???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?大角�????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��度 [rad/s]
//    float a_max;     // ???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?大角加速度 [rad/s^2]
//    bool  is_finished;
//} JointProfile1D;
//
//typedef struct {
//    int32_t current_step;      // 現在step
//    int32_t target_step;       // 目標step
//    float   step_rate_cmd;     // 目標step速度 [step/s]
//    float   step_rate_max;     // ???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?大step速度 [step/s]
//    float   kp_step;           // 位置誤差(error)→�????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��度(step_rate_cmd) 変換ゲイン
//    float   phase_accum;       // accumletor
//} StepFollower1D;

static const int32_t polling1_ms = 200;
int32_t polling1_tmp_last = 0;
static const int32_t polling2_ms = 1 * 1000;
int32_t polling2_tmp_last = 0;
static const int32_t polling3_ms = 7 * 1000;
int32_t polling3_tmp_last = 0;

int32_t now = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



HAL_StatusTypeDef debag_state = HAL_OK;

// {現在角度 [rad],現在角�????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��度 [rad/s],目標角度 [rad],???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?大角�????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��度 [rad/s],???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?大角加速度 [rad/s^2]}
//JointProfile1D JointProfile[3] = {
//	{0, 0, 0, 1.5f, 5.0f, false},//axis1
//	{0, 0, 0, 1.0f, 0.5f, false},//axis2
//	{0, 0, 0, 1.0f, 0.5f, false},//axis3
//};

//{現在step, 目標step, 目標step速度 [step/s], // ???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?大step速度 [step/s], 位置誤差(error)→�????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��度(step_rate_cmd) 変換ゲイン, accumletor
//StepFollower1D StepFollower[3] = {
//	{0, 0, 0.0f, 10000.0f, 1.0f, 0.0f},
//	{0, 0, 0.0f, 1000.0f, 1.0f, 0.0f},
//	{0, 0, 0.0f, 1000.0f, 1.0f, 0.0f},
//};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void DWT_Init(void);
void delay_us(uint32_t us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void DWT_Init(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us){
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}


HAL_StatusTypeDef axis_control_timer_init(void)
{
//    uint32_t tim_clk_hz;
    uint32_t psc;
    uint32_t arr;
    float target_freq_hz;

    target_freq_hz = AXIS_CONTROL_FREQ_HZ;
//    tim_clk_hz = 90000000U;
    psc = 89U;
    arr = (uint32_t)(1000000.0f / target_freq_hz - 1.0f);

    __HAL_TIM_SET_PRESCALER(&htim6, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_SET_COUNTER(&htim6, 0);

    return HAL_OK;
}

extern UART_HandleTypeDef huart1;

//static sts_bus_t g_sts_bus;
//static sts3215_t g_servo1;
//void app_sts_test(void){
//    sts_bus_init(&g_sts_bus, &huart1, 20, 20);
//    sts3215_init(&g_servo1, &g_sts_bus, 13);
//
//    sts_status_t ret;
//    uint16_t pos = 0;
//    uint8_t temp = 0;
//
//    ret = sts3215_ping(&g_servo1);
//    if (ret == STS_OK) {
//        printf("PING OK\r\n");
//    } else {
//        printf("PING NG: %d\r\n", ret);
//        return;
//    }
//
//    // �?: Present Position のアドレスは手�??の表に合わせて変更
//    ret = sts3215_read_u16(&g_servo1, 56, &pos);
//    if (ret == STS_OK) {
//        printf("POS = %u\r\n", pos);
//    } else {
//        printf("READ POS NG: %d\r\n", ret);
//    }
//
//    // �?: 温度アドレスも手�?の表に合わせる
//    ret = sts3215_read_u8(&g_servo1, 63, &temp);
//    if (ret == STS_OK) {
//        printf("TEMP = %u\r\n", temp);
//    } else {
//        printf("READ TEMP NG: %d\r\n", ret);
//    }
//}

//void app_servo_move_test(void)
//{
//    sts_bus_init(&g_sts_bus, &huart1, 20, 20);
//    sts3215_init(&g_servo1, &g_sts_bus, 11);
//    uint16_t pos;
//
//    printf("READ POS\r\n");
//
//    sts3215_read_u16(
//            &g_servo1,
//            56,
//            &pos);
//
//    printf("POS = %u\r\n", pos);
//
//    printf("TORQUE ON\r\n");
//
//    sts3215_set_torque_enable(
//            &g_servo1,
//            true);
//
//    HAL_Delay(100);
//
//    printf("MOVE +1000\r\n");
//
//    sts3215_set_goal_position(
//            &g_servo1,
//            pos + 1000);
//}

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
  DWT_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  printf("\n\n boot\r\n");
  pc_interface_init(&huart2);

//  if(stepper_init() 				!= HAL_OK) Error_Handler();
//  if(axis_init() 					!= HAL_OK) Error_Handler();
  if (app_init() 					!= HAL_OK) Error_Handler();

  if (axis_control_timer_init() 	!= HAL_OK) Error_Handler();
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
//  HAL_Delay(1000);
//  printf("\n initialize \r\n");


//  printf("\n\n\n boot\r\n");
  HAL_Delay(1000);



//  printf("start_homing\r\n");
//  app_start_homing_all();
////  HomingState_t state1;
//  AppHomingSeqState_t state;
//  while(1){//
//	  app_get_homing_seq_state(&state);
////	  homing_control_get_state(1, &state1);
//
////	  printf("%d %d\r\n",state, state1);
//
//	  if(state == APP_HOMING_SEQ_IDLE){
//		  printf("done\r\n");
//		  break;
//	  }else if(state == APP_HOMING_SEQ_ERROR){
//		  printf("homing_error\r\n");
//		  Error_Handler();
//	  }
//	  delay_us(100 * 1000);
//  }
  if(app_set_control_source_pc() != HAL_OK){
	  Error_Handler();
  }
  printf("APP MODE: PC_CONTROL\r\n");

//  app_sts_test();

//  app_servo_move_test();

  int flag = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  now = HAL_GetTick();

	  if(now - polling1_tmp_last >= polling1_ms && 0){
		  polling1_tmp_last = now;
		  int32_t step1;
		  int32_t step2;
		  int32_t step3;
		  stepper_get_current_step(1, &step1);
		  stepper_get_current_step(2, &step2);
		  stepper_get_current_step(3, &step3);
		  printf("%ld %ld %ld\r\n", step1,step2,step3);
	  }



	  if(now - polling2_tmp_last >= polling2_ms && 0){
		  polling2_tmp_last = now;

	  }


	  if(now - polling3_tmp_last >= polling3_ms && 0){
		  polling3_tmp_last = now;
		  if(flag == 0){
			  flag = 1;
			  printf("a\r\n");
//			  if(axis_set_target_step(1, 50000, 10000.0f) != HAL_OK)Error_Handler();
//			  if(app_move_to_step(1, 5000, 5000.0f, 8000.0f) != HAL_OK)Error_Handler();
//			  if(axis_set_motion_target(1, 5000, 5000.0f, 8000.0f) != HAL_OK)Error_Handler();
//			  if (app_move_joint_target(deg_to_rad(20.0f), deg_to_rad(20.0f), deg_to_rad(10.0f),
//					  	  	  	  	    deg_to_rad(20.0f), deg_to_rad(20.0f), deg_to_rad(10.0f),
//										deg_to_rad(20.0f), deg_to_rad(20.0f), deg_to_rad(10.0f)) != HAL_OK)
//			  {
//			      Error_Handler();
//			  }
//			  if (sync_motion_move_joint_target(deg_to_rad(0.0f), deg_to_rad(0.0f), deg_to_rad(0.0f),
//			                                 deg_to_rad(20.0f),
//											 deg_to_rad(20.0f)) != HAL_OK)
//			  {
//			      Error_Handler();
//			  }
			  if (joint_mapper_set_target_rad(deg_to_rad(0.0f), deg_to_rad(0.0f), deg_to_rad(0.0f),
					                          deg_to_rad(0.0f), deg_to_rad(0.0f), deg_to_rad(0.0f),
			                                  deg_to_rad(20.0f),
											  deg_to_rad(20.0f)) != HAL_OK)
			  {
			      Error_Handler();
			  }

		  }else if(flag == 1){
			  flag = 0;
			  printf("b\r\n");
//			  if(axis_set_target_step(1, 0, 10000.0f) != HAL_OK)Error_Handler();
//			  if(app_move_to_step(1, 0, 5000.0f, 8000.0f) != HAL_OK)Error_Handler();
//			  if(axis_set_motion_target(1, 0, 5000.0f, 8000.0f) != HAL_OK)Error_Handler();
//			  if (app_move_joint_target(deg_to_rad(0.0f), deg_to_rad(20.0f), deg_to_rad(10.0f),
//					  	  	  	  	    deg_to_rad(0.0f), deg_to_rad(20.0f), deg_to_rad(10.0f),
//										deg_to_rad(0.0f), deg_to_rad(20.0f), deg_to_rad(10.0f)) != HAL_OK)
//			  {
//			      Error_Handler();
//			  }
//			  if (sync_motion_move_joint_target(deg_to_rad(45.0f), deg_to_rad(45.0f), deg_to_rad(45.0f),
//			                                 deg_to_rad(20.0f),
//											 deg_to_rad(20.0f)) != HAL_OK)
//			  {
//			      Error_Handler();
//			  }
			  if (joint_mapper_set_target_rad(deg_to_rad(45.0f), deg_to_rad(45.0f), deg_to_rad(45.0f),
					                          deg_to_rad(45.0f), deg_to_rad(45.0f), deg_to_rad(45.0f),
			                                  deg_to_rad(20.0f),
											  deg_to_rad(20.0f)) != HAL_OK)
			  {
			      Error_Handler();
			  }

		  }else if(flag == 2){
			  flag = 3;
//			  if(set_dir(3, DIR_NEGATIVE) != HAL_OK)Error_Handler();
//			  if(step_timer_set_rate(3, 5000.0f) != HAL_OK)Error_Handler();
			  printf("c\r\n");
		  }else if(flag == 3){
			  flag = 0;
//			  if(step_timer_set_rate(3, 0.0f) != HAL_OK)Error_Handler();
			  printf("d\r\n");
		  }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM6){
    	if (app_update() != HAL_OK) Error_Handler();
//    	pc_interface_update();
//    	if (axis_update_all() != HAL_OK) Error_Handler();

//        if(axis_update(1) != HAL_OK) Error_Handler();
//        if(axis_update(2) != HAL_OK) Error_Handler();
//        if(axis_update(3) != HAL_OK) Error_Handler();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    pc_interface_uart_rx_callback(huart);
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
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  delay_us(150 * 1000);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  delay_us(150 * 1000);
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
