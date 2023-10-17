/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5047p.h"
#include "can.h"
#include "can_util.h"
#include "caw_status.h"
#include "drv8323.h"
#include "log.h"
#include "motor.h"
#include "pid.h"
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
/* USER CODE BEGIN Variables */
extern AS5047P_T g_sensor;
DRV8323_T driver;
volatile uint8_t drv_health_state;
volatile uint8_t drv_init_state;
/* USER CODE END Variables */
osThreadId stateTaskHandle;
osThreadId driverTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartStateTask(void const *argument);
void StartDriverTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  drv_health_state = 0;
  drv_init_state = 0;
  CAN_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of stateTask */
  osThreadDef(stateTask, StartStateTask, osPriorityNormal, 0, 128);
  stateTaskHandle = osThreadCreate(osThread(stateTask), NULL);

  /* definition and creation of driverTask */
  osThreadDef(driverTask, StartDriverTask, osPriorityNormal, 0, 512);
  driverTaskHandle = osThreadCreate(osThread(driverTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartStateTask */
/**
 * @brief Function implementing the stateTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStateTask */
void StartStateTask(void const *argument) {
  /* USER CODE BEGIN StartStateTask */
  /* Infinite loop */
  for (;;) {
    if (drv_init_state) {
      uint16_t r0, r1;
      if ((!DRV8323_GetFaultStatus(&driver, &r0) && !r0) &&
          (!DRV8323_GetVGSStatus(&driver, &r1) && !r1)) {
        drv_health_state = 1;
      } else {
        // CAW_LOG_ERROR("DRV FAULT FAULT-REG: %X VGS-REG: %X", r0, r1);
        drv_health_state = 0;
      }
    }
    GPIO_PinState sta = HAL_GPIO_ReadPin(MCU_STA_GPIO_Port, MCU_STA_Pin);
    HAL_GPIO_WritePin(MCU_STA_GPIO_Port, MCU_STA_Pin, !sta);
    if (drv_health_state)
      HAL_GPIO_WritePin(DRV_STA_GPIO_Port, DRV_STA_Pin, !sta);
    else
      HAL_GPIO_WritePin(DRV_STA_GPIO_Port, DRV_STA_Pin, GPIO_PIN_RESET);
    osDelay(250);
  }
  /* USER CODE END StartStateTask */
}

/* USER CODE BEGIN Header_StartDriverTask */
/**
 * @brief Function implementing the driverTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDriverTask */
void StartDriverTask(void const *argument) {
  /* USER CODE BEGIN StartDriverTask */
  HAL_Delay(1000);
  // 初始化DRV8323
  if (!DRV8323_Init(&driver, &hspi3)) {
    drv_init_state = 1;
    DRV8323_ShowRegs(&driver);
    CAW_LOG_INFO("DRV8323 init ok...");
  } else {
    // 初始化失败退出任务
    CAW_LOG_FAIL("DRV8323 init failed...");
  }

  MOTOR_T motor;
  DRIVER_6PWM_T driver_6;
  DRIVER_6PWM_Init(&driver_6, &htim1, PWM_PERIOD);
  MOTOR_Init(&motor, 14);
  MOTOR_LinkDriver(&motor, &driver_6);

  driver_6.voltage_power_supply = 24.0f;
  driver_6.voltage_limit = 20.0f;
  motor.voltage_limit = 20.0f;
  motor.voltage_sensor_align = 3.0f;

  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20.0f;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.01;
  motor.PID_angle.P = 10.0f;

  motor.shaft_velocity_sp = 6.28f;

  float target_angle = 0;

  int count = 0;

  MOTOR_AlignSensor(&motor);

  /* Infinite loop */
  for (;;) {
    if (count++ >= 500) {
      count = 0;
      target_angle = target_angle == 0 ? 6.28 : 0;
    }
    MOTOR_Step(&motor);
    MOTOR_Move(&motor, target_angle);
    // MOTOR_VelocityOpenloop(&motor, 6.28);
    // CURRENT_SENSOR_Update(&(motor.current_sensor));
    // motor.current.q =
    //     LOWPASS_FILTER_Calc(&(motor.LPF_current_q), motor.current.q);
    // if (count++ > 30) {
    //   count = 0;
    //   CAW_LOG_DEBUG("q: %f", motor.current.q);
    // }
    osDelay(1);
  }
  /* USER CODE END StartDriverTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
