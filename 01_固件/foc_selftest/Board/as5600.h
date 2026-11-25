#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// AS5600 I2C 7-bit 地址 (0x36)
#define AS5600_I2C_ADDR      0x36

// 寄存器地址（请参考数据手册）
#define AS5600_REG_ZMCO      0x00
#define AS5600_REG_ZPOS_H    0x01
#define AS5600_REG_ZPOS_L    0x02
#define AS5600_REG_MPOS_H    0x03
#define AS5600_REG_MPOS_L    0x04
#define AS5600_REG_MANG_H    0x05
#define AS5600_REG_MANG_L    0x06
#define AS5600_REG_CONF      0x07
#define AS5600_REG_CONF_H    0x07   // 高字节
#define AS5600_REG_CONF_L    0x08   // 低字节
#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_REG_RAW_ANGLE_L 0x0D
#define AS5600_REG_ANGLE_H   0x0E
#define AS5600_REG_ANGLE_L   0x0F
#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_AGC       0x1A
#define AS5600_REG_MAG_H     0x1B
#define AS5600_REG_MAG_L     0x1C
#define AS5600_REG_BURN      0xFF

// 状态位
#define AS5600_STATUS_MD     (1 << 0)  // 磁铁检测
#define AS5600_STATUS_ML     (1 << 1)  // 磁场太弱（AGC 溢出）
#define AS5600_STATUS_MH     (1 << 2)  // 磁场太强（AGC 溢出）

// 烧写命令
#define AS5600_BURN_ANGLE    0x80
#define AS5600_BURN_SETTING  0x40

// 初始化 I2C 句柄（外部定义）
extern I2C_HandleTypeDef hi2c1;

// 函数声明
uint8_t AS5600_ReadReg(uint8_t reg, uint8_t *pData, uint16_t size);
uint8_t AS5600_WriteReg(uint8_t reg, uint8_t *pData, uint16_t size);
uint8_t AS5600_ReadAngle(uint16_t *angle);              // 读取缩放角度 (0~4095)
uint8_t AS5600_ReadRawAngle(uint16_t *raw);             // 读取原始角度
uint8_t AS5600_ReadStatus(uint8_t *status);
uint8_t AS5600_ReadAGC(uint8_t *agc);
uint8_t AS5600_ReadMagnitude(uint16_t *mag);
uint8_t AS5600_ReadZMCO(uint8_t *zmco);                 // 读取烧写次数
void AS5600_GetAngleDegree(float *degree);              // 直接返回角度（度）

// 配置相关（可选）
uint8_t AS5600_WriteZPOS(uint16_t zpos);
uint8_t AS5600_WriteMPOS(uint16_t mpos);
uint8_t AS5600_WriteMANG(uint16_t mang);
uint8_t AS5600_WriteCONF(uint16_t conf);
uint8_t AS5600_BurnAngle(void);
uint8_t AS5600_BurnSetting(void);

#endif
