/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"

#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "drv8323.h"
#include "log.h"
#include "motor.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
extern volatile uint8_t drv_health_state;
extern volatile uint8_t drv_init_state;
extern volatile uint8_t foc_run_state;
extern float foc_target;
extern MOTOR_T motor;
extern DRV8323_T driver;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles CAN1 RX0 interrupt.
 */
void CAN1_RX0_IRQHandler(void) {
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void) {
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(DRV_FAULT_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun
 * error interrupts.
 */
void TIM6_DAC_IRQHandler(void) {
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // DRV驱动器故障
  if (DRV_FAULT_Pin == GPIO_Pin) {
    uint16_t r0, r1;
    DRV8323_GetFaultStatus(&driver, &r0);
    DRV8323_GetVGSStatus(&driver, &r1);
    CAW_LOG_ERROR("DRV FAULT FAULT-REG: %X VGS-REG: %X", r0, r1);
    if (!drv_init_state || !drv_health_state) return;
    drv_health_state = 0;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t data[8];
  uint8_t reply_data[8];
  float ret_value = 0.0f;
  uint8_t accuracy = 0;

  // 应答数据
  CAN_TxHeaderTypeDef txHeader;
  txHeader.DLC = sizeof(reply_data);
  txHeader.StdId = 0x0000;
  txHeader.RTR = 0x0000;
  txHeader.IDE = 0x0000;

  HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &rxHeader, data);
  if (CAN1 == hcan->Instance) {
    PROTOCOL_T p;
    PROTOCOL_Unpack(&p, data);
    if (p.version != PROTOCOL_VERSION) return;

    //! 设置启动或停止状态
    if (p.cmd == PROTOCOL_STOP) {
      if (p.action == PROTOCOL_SET) {
        foc_run_state = 0;
        CAW_LOG_DEBUG("STOP => foc_run_state: %d", foc_run_state);
      }
    } else if (p.cmd == PROTOCOL_START) {
      if (p.action == PROTOCOL_SET) {
        foc_run_state = 1;
        CAW_LOG_DEBUG("START => foc_run_state: %d", foc_run_state);
      }
    }

    //! 控制类型设置
    else if (p.cmd == PROTOCOL_CONTROL_TYPE) {
      if (p.action == PROTOCOL_SET) {
        int type = (int)p.data;
        if (type == PROTOCOL_TORQUE) {
          motor.controller = TORQUE;
        } else if (type == PROTOCOL_VELOCITY) {
          motor.controller = VELOCITY;
        } else if (type == PROTOCOL_ANGLE) {
          motor.controller = ANGLE;
        } else if (type == PROTOCOL_VELOCITY_OPENLOOP) {
          motor.controller = VELOCITY_OPENLOOP;
        } else if (type == PROTOCOL_ANGLE_OPENLOOP) {
          motor.controller = ANGLE_OPENLOOP;
        }
      }
    }

    //! 设置目标速度值（角度控制时使用）
    else if (p.cmd == PROTOCOL_VELOCITY_SP) {
      if (p.action == PROTOCOL_SET) {
        motor.shaft_velocity_sp = p.data;
      }
    }

    //! 设置目标值
    else if (p.cmd == PROTOCOL_TARGET) {
      if (p.action == PROTOCOL_SET) {
        foc_target = p.data;
      }
    }

    //! 获取电流，速度，角度
    else if (p.cmd == PROTOCOL_CURRENT_VALUE) {
      if (p.action == PROTOCOL_GET) {
        ret_value = motor.current.q;
        accuracy = 2;
      }
    }

    PROTOCOL_Pack(PROTOCOL_REPLY, PROTOCOL_NONE, accuracy, ret_value,
                  reply_data);
    HAL_CAN_AddTxMessage(hcan, &txHeader, reply_data, CAN_TX_MAILBOX0);
  }
}
/* USER CODE END 1 */
