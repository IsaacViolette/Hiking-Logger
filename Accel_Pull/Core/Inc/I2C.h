#ifndef __STM32L476G_DISCOVERY_I2C_H
#define __STM32L476G_DISCOVERY_I2C_H

#include <stdint.h>
#include <stddef.h>
#include "stm32l4xx.h"

// PB 10 <--> AF4 (I2C2_SCL) 29
// PB 11 <--> AF4 (I2C2_SDA) 30
#define I2C_SCL_PIN  10
#define I2C_SDA_PIN  11
#define I2C_PORT     GPIOB

#define READ_FROM_SLAVE 1
#define WRITE_TO_SLAVE  0

int8_t I2C_Start(I2C_TypeDef * I2Cx, uint32_t DevAddress, uint8_t Size, uint8_t Direction);
void I2C_Stop(I2C_TypeDef * I2Cx);
void I2C_WaitLineIdle(I2C_TypeDef * I2Cx);
int8_t I2C_SendData(I2C_TypeDef * I2Cx, uint8_t DeviceAddress, uint8_t *pData, uint8_t Size);
int8_t I2C_ReceiveData(I2C_TypeDef * I2Cx, uint8_t DeviceAddress, uint8_t *pData, uint8_t Size);
void CODEC_Initialization(void);


#endif /* __STM32L476G_DISCOVERY_I2C_H */
