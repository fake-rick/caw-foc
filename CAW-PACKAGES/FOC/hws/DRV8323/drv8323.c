/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-08-30 00:52:58
 * @LastEditors: Rick
 * @LastEditTime: 2023-10-13 17:48:41
 * @Description:
 */

#include "drv8323.h"

#include <stdio.h>
#include <string.h>

#include "caw_status.h"
#include "gpio.h"
#include "log.h"
#include "spi.h"
#include "stdint.h"

// SPI通讯NSS
#define _NSS(x) HAL_GPIO_WritePin(DRV_SPI_NSS_GPIO_Port, DRV_SPI_NSS_Pin, x)
// 放大器较准输入，逻辑高时执行自动偏移较准，校准后需要拉低
#define _ENABLE(x) HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, x)
// 低电平进入睡眠模式，高电平使能，8~40uS低脉冲重置故障
#define _CAL(x) HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, x)

// #define _INLA(x) HAL_GPIO_WritePin(DRV_INLA_GPIO_Port, DRV_INLA_Pin, x)
// #define _INLB(x) HAL_GPIO_WritePin(DRV_INLB_GPIO_Port, DRV_INLB_Pin, x)
// #define _INLC(x) HAL_GPIO_WritePin(DRV_INLC_GPIO_Port, DRV_INLC_Pin, x)
/**
 * @description: 制作指令
 * --------------------------
 * | R/W | ADDRESS |  DATA  |
 * | B15 | B14-B11 | B10-B0 |
 * --------------------------
 * @param {uint16_t} rw 写入[0] 读取[1]
 * @param {uint16_t} addr 地址
 * @param {uint16_t} data 数据
 * @return {*}
 */
static inline uint16_t _MakeWriteData(uint16_t rw, uint16_t addr,
                                      uint16_t data) {
  return (rw << 15) | ((addr & 0xF) << 11) | (data & 0x7FF);
}

/**
 * @description: 处理接收到的数据
 * @param {uint16_t} data
 * @return {*}
 */
static inline uint16_t _MakeRecvData(uint16_t data) {
  return ((data << 5) >> 5);
}

/**
 * @description: SPI读写处理
 * @param {uint16_t} tx
 * @param {uint16_t*} rx
 * @return {*}
 */
static inline int _ReadWrite(DRV8323_T* drv, uint16_t tx, uint16_t* rx) {
  _NSS(0);
  int ret = HAL_SPI_TransmitReceive(drv->hspi, &tx, rx, 1, HAL_MAX_DELAY);
  _NSS(1);
  return HAL_OK == ret ? CAW_OK : CAW_ERR;
}

/**
 * @description: 设置寄存器值
 * @param {uint16_t} addr
 * @param {uint16_t} value
 * @return {*} [CAW_OK, CAW_ERR]
 */
int _SetReg(DRV8323_T* drv, uint16_t addr, uint16_t value) {
  uint16_t ret;
  uint16_t cmd = _MakeWriteData(0, addr, value);
  if (_ReadWrite(drv, cmd, &ret)) {
    return CAW_ERR;
  }
  cmd = _MakeWriteData(1, addr, 0);
  if (_ReadWrite(drv, cmd, &ret)) {
    return CAW_ERR;
  }
  if (_MakeRecvData(ret) != _MakeRecvData(value)) {
    return CAW_ERR;
  }
  return CAW_OK;
}

/**
 * @description: 获取寄存器值
 * @param {uint16_t} addr
 * @param {uint16_t*} value
 * @return {*}
 */
int _GetReg(DRV8323_T* drv, uint16_t addr, uint16_t* value) {
  uint16_t cmd = _MakeWriteData(1, addr, 0);
  if (_ReadWrite(drv, cmd, value)) {
    return CAW_ERR;
  }
  _MakeRecvData(*value);
  return CAW_OK;
}

/**
 * @description: 初始化DRV8323
 * @param {DRV8323_T*} drv
 * @return {*}
 */
int DRV8323_Init(DRV8323_T* drv, SPI_HandleTypeDef* hspi) {
  drv->hspi = hspi;
  _ENABLE(1);
  HAL_Delay(500);
  // 配置寄存器
  if (_SetReg(drv, DRV8323_DRIVER_CONTROL_REG, 0b00000000000)) {
    return CAW_ERR;
  }
  if (_SetReg(drv, DRV8323_GATE_DRIVE_HS_REG, 0b01111111111)) {
    return CAW_ERR;
  }
  if (_SetReg(drv, DRV8323_GATE_DRIVE_LS_REG, 0b11111111111)) {
    return CAW_ERR;
  }
  if (_SetReg(drv, DRV8323_OCP_CONTROL_REG, 0b01001011001)) {
    return CAW_ERR;
  }
  if (_SetReg(drv, DRV8323_CSA_CONTROL_REG, 0b01011000011)) {
    return CAW_ERR;
  }

  // 较准放大器
  _CAL(1);
  HAL_Delay(100);
  _CAL(0);
  HAL_Delay(100);

  return CAW_OK;
}

int DRV8323_GetFaultStatus(DRV8323_T* drv, uint16_t* r0) {
  if (_GetReg(drv, DRV8323_FAULT_STATUS_REG, r0)) {
    return CAW_ERR;
  }
  return CAW_OK;
}

int DRV8323_GetVGSStatus(DRV8323_T* drv, uint16_t* r1) {
  if (_GetReg(drv, DRV8323_VGS_STATUS_REG, r1)) {
    return CAW_ERR;
  }
  return CAW_OK;
}

void DRV8323_ShowRegs(DRV8323_T* drv) {
  uint16_t r0, r1, r2, r3, r4, r5, r6;
  if (_GetReg(drv, DRV8323_FAULT_STATUS_REG, &r0)) {
    return;
  }
  if (_GetReg(drv, DRV8323_VGS_STATUS_REG, &r1)) {
    return;
  }
  if (_GetReg(drv, DRV8323_DRIVER_CONTROL_REG, &r2)) {
    return;
  }
  if (_GetReg(drv, DRV8323_GATE_DRIVE_HS_REG, &r3)) {
    return;
  }
  if (_GetReg(drv, DRV8323_GATE_DRIVE_LS_REG, &r4)) {
    return;
  }
  if (_GetReg(drv, DRV8323_OCP_CONTROL_REG, &r5)) {
    return;
  }
  if (_GetReg(drv, DRV8323_CSA_CONTROL_REG, &r6)) {
    return;
  }
  char buf[512];
  sprintf(buf,
          "\r\n********** REG INFO START **********\r\n\r\n"
          "DRV8323_FAULT_STATUS_REG:\t0x%X\r\n"
          "DRV8323_VGS_STATUS_REG:\t\t0x%X\r\n"
          "DRV8323_DRIVER_CONTROL_REG:\t0x%X\r\n"
          "DRV8323_DRIVER_CONTROL_REG:\t0x%X\r\n"
          "DRV8323_GATE_DRIVE_HS_REG:\t0x%X\r\n"
          "DRV8323_GATE_DRIVE_LS_REG:\t0x%X\r\n"
          "DRV8323_OCP_CONTROL_REG:\t0x%X\r\n"
          "DRV8323_CSA_CONTROL_REG:\t0x%X\r\n"
          "\r\n*********** REG INFO END ***********\r\n\r\n",
          r0, r1, r2, r3, r4, r5, r6);
  CAW_LOG_RAW(buf);
}