#ifndef __DRV8323_H__
#define __DRV8323_H__

#include "spi.h"

// Fault Status 1
#define DRV8323_FAULT_STATUS_REG 0x00
// VGS Status 2
#define DRV8323_VGS_STATUS_REG 0x01
// 驱动控制寄存器
#define DRV8323_DRIVER_CONTROL_REG 0x02
// 高桥臂栅极控制寄存器
#define DRV8323_GATE_DRIVE_HS_REG 0x03
// 低桥臂栅极控制寄存器
#define DRV8323_GATE_DRIVE_LS_REG 0x04
// 过流保护寄存器
#define DRV8323_OCP_CONTROL_REG 0x05
// 电流检测放大器控制寄存器
#define DRV8323_CSA_CONTROL_REG 0x06

typedef struct _DRV8323_T {
  SPI_HandleTypeDef* hspi;
} DRV8323_T;

int DRV8323_Init(DRV8323_T* drv, SPI_HandleTypeDef* hspi);
void DRV8323_ShowRegs(DRV8323_T* drv);
int DRV8323_GetFaultStatus(DRV8323_T* drv, uint16_t* r0);
int DRV8323_GetVGSStatus(DRV8323_T* drv, uint16_t* r1);

#endif