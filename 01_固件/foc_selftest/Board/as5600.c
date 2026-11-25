#include "as5600.h"
#include "stm32f4xx_hal.h"

// 外部 I2C 句柄（需在 main.c 中定义并初始化）
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief 向 AS5600 的寄存器写入数据（单字节或连续）
 * @param reg  寄存器地址
 * @param pData 数据指针
 * @param size  字节数
 * @retval HAL 状态
 */
uint8_t AS5600_WriteReg(uint8_t reg, uint8_t *pData, uint16_t size)
{
    // 发送：设备地址（写）+ 寄存器地址 + 数据
    return HAL_I2C_Mem_Write(&hi2c1, AS5600_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                             pData, size, HAL_MAX_DELAY);
}

/**
 * @brief 从 AS5600 寄存器读取数据
 * @param reg  寄存器地址
 * @param pData 数据缓冲区
 * @param size  读取字节数
 * @retval HAL 状态
 */
uint8_t AS5600_ReadReg(uint8_t reg, uint8_t *pData, uint16_t size)
{
    return HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                            pData, size, HAL_MAX_DELAY);
}

/**
 * @brief 读取缩放后的角度（ANGLE 寄存器，12位）
 * @param angle 输出角度值 (0~4095)
 * @retval HAL_OK 成功，其他失败
 */
uint8_t AS5600_ReadAngle(uint16_t *angle)
{
    uint8_t data[2];
    if (AS5600_ReadReg(AS5600_REG_ANGLE_H, data, 2) != HAL_OK)
        return HAL_ERROR;
    *angle = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

/**
 * @brief 读取原始角度（RAW ANGLE，未经缩放和滤波）
 * @param raw 输出原始角度值
 * @retval HAL_OK 成功，其他失败
 */
uint8_t AS5600_ReadRawAngle(uint16_t *raw)
{
    uint8_t data[2];
    if (AS5600_ReadReg(AS5600_REG_RAW_ANGLE_H, data, 2) != HAL_OK)
        return HAL_ERROR;
    *raw = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

/**
 * @brief 读取状态寄存器
 * @param status 状态字节
 * @retval HAL_OK 成功，其他失败
 */
uint8_t AS5600_ReadStatus(uint8_t *status)
{
    return AS5600_ReadReg(AS5600_REG_STATUS, status, 1);
}

/**
 * @brief 读取 AGC 值（自动增益控制）
 * @param agc AGC 值 (0~255)
 * @retval HAL_OK 成功，其他失败
 */
uint8_t AS5600_ReadAGC(uint8_t *agc)
{
    return AS5600_ReadReg(AS5600_REG_AGC, agc, 1);
}

/**
 * @brief 读取磁场强度（MAGNITUDE 寄存器，12位）
 * @param mag 强度值
 * @retval HAL_OK 成功，其他失败
 */
uint8_t AS5600_ReadMagnitude(uint16_t *mag)
{
    uint8_t data[2];
    if (AS5600_ReadReg(AS5600_REG_MAG_H, data, 2) != HAL_OK)
        return HAL_ERROR;
    *mag = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

/**
 * @brief 读取 ZMCO（烧写次数）
 * @param zmco 返回 0~3
 * @retval HAL_OK 成功，其他失败
 */
uint8_t AS5600_ReadZMCO(uint8_t *zmco)
{
    uint8_t val;
    if (AS5600_ReadReg(AS5600_REG_ZMCO, &val, 1) != HAL_OK)
        return HAL_ERROR;
    *zmco = val & 0x03;  // 低2位
    return HAL_OK;
}

/**
 * @brief 获取角度（浮点数，单位度）
 * @param degree 输出角度 (0.0~359.9)
 */
void AS5600_GetAngleDegree(float *degree)
{
    uint16_t angle;
    if (AS5600_ReadAngle(&angle) == HAL_OK) {
        *degree = (float)angle * 360.0f / 4096.0f;
    } else {
        *degree = -1.0f; // 错误值
    }
}

// -------------------- 配置寄存器写入（可选） --------------------

/**
 * @brief 写入 ZPOS（起始位置，12位）
 */
uint8_t AS5600_WriteZPOS(uint16_t zpos)
{
    uint8_t data[2] = { (uint8_t)(zpos >> 8), (uint8_t)(zpos & 0xFF) };
    return AS5600_WriteReg(AS5600_REG_ZPOS_H, data, 2);
}

/**
 * @brief 写入 MPOS（停止位置，12位）
 */
uint8_t AS5600_WriteMPOS(uint16_t mpos)
{
    uint8_t data[2] = { (uint8_t)(mpos >> 8), (uint8_t)(mpos & 0xFF) };
    return AS5600_WriteReg(AS5600_REG_MPOS_H, data, 2);
}

/**
 * @brief 写入 MANG（最大角度范围，12位）
 */
uint8_t AS5600_WriteMANG(uint16_t mang)
{
    uint8_t data[2] = { (uint8_t)(mang >> 8), (uint8_t)(mang & 0xFF) };
    return AS5600_WriteReg(AS5600_REG_MANG_H, data, 2);
}

/**
 * @brief 写入 CONF 寄存器（16位）
 * @param conf 配置值（高字节在 0x07，低字节在 0x08）
 */
uint8_t AS5600_WriteCONF(uint16_t conf)
{
    uint8_t data[2] = { (uint8_t)(conf >> 8), (uint8_t)(conf & 0xFF) };
    // 从 0x07 开始连续写 2 字节
    return AS5600_WriteReg(AS5600_REG_CONF_H, data, 2);
}

/**
 * @brief 执行 BURN_ANGLE 命令（永久保存 ZPOS 和 MPOS）
 * @note 最多可执行 3 次
 */
uint8_t AS5600_BurnAngle(void)
{
    uint8_t cmd = AS5600_BURN_ANGLE;
    return AS5600_WriteReg(AS5600_REG_BURN, &cmd, 1);
}

/**
 * @brief 执行 BURN_SETTING 命令（永久保存 MANG 和 CONF）
 * @note 只能执行 1 次，且 ZPOS/MPOS 未被烧写时可用
 */
uint8_t AS5600_BurnSetting(void)
{
    uint8_t cmd = AS5600_BURN_SETTING;
    return AS5600_WriteReg(AS5600_REG_BURN, &cmd, 1);
}
