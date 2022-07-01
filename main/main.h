/**
 * @file main.h
 * @author PeerGum (phil@peergum.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-30
 * 
 * © Copyright 2022, PeerGum.
 * 
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include "driver/i2c.h"

#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 250000

#define NAU7802_I2C_PORT (I2C_NUM_MAX - 1)
#define NAU7802_INT_PIN GPIO_NUM_NC

#endif // __MAIN_H__