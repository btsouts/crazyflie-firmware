/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file ina260.h
 * Driver for the ina260 XXX.
 *
 */
#ifndef INA260_H
#define INA260_H

#include <stdbool.h>
#include "i2cdev.h"

#define MEASURE_CURRENT
#define MEASURE_VOLTAGE

/*!
 * @file Adafruit_INA260.h
 *
 * This is a library for the Adafruit INA260 breakout board
 * ----> https://www.adafruit.com/products/904
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

/** default I2C address **/
// #define INA260_ADDRESS (0x40) // 1000000 (A0+A1=GND)

#define INA260_I2CADDR_DEFAULT  0x40 ///< INA260 default i2c address
#define INA260_REG_CONFIG       0x00 ///< Configuration register
#define INA260_REG_CURRENT      0x01 ///< Current measurement register (signed) in mA
#define INA260_REG_BUSVOLTAGE   0x02 ///< Bus voltage measurement register in mV
#define INA260_REG_POWER        0x03 ///< Power calculation register in mW
#define INA260_REG_MASK_ENABLE  0x06 ///< Interrupt/Alert setting and checking register
#define INA260_REG_ALERT_LIMIT  0x07 ///< Alert limit value register
#define INA260_REG_MFG_UID      0xFE ///< Manufacturer ID Register
#define INA260_REG_DIE_UID      0xFF ///< Die ID and Revision Register

/**
 * @brief Mode options.
 *
 * Allowed values for setMode.
 */
typedef enum _mode {
  INA260_MODE_SHUTDOWN = 0x00, /**< SHUTDOWN: Minimize quiescient current and turn
                                    off current into the device inputs. Set another
                                    mode to exit shutown mode **/
  INA260_MODE_TRIGGERED  = 0x03, /**< TRIGGERED: Trigger a one-shot measurement
                                      of current and bus voltage. Set the TRIGGERED
                                      mode again to take a new measurement **/
  INA260_MODE_CONTINUOUS = 0x07, /**< CONTINUOUS: (Default) Continuously update
                                      the current, bus voltage and power registers
                                      with new measurements **/
} INA260_MeasurementMode;

/**
 * @brief Conversion Time options.
 *
 * Allowed values for setCurrentConversionTime and setVoltageConversionTime.
 */
typedef enum _conversion_time {
  INA260_TIME_140_us, ///< Measurement time: 140us
  INA260_TIME_204_us, ///< Measurement time: 204us
  INA260_TIME_332_us, ///< Measurement time: 332us
  INA260_TIME_558_us, ///< Measurement time: 558us
  INA260_TIME_1_1_ms, ///< Measurement time: 1.1ms (Default)
  INA260_TIME_2_116_ms, ///< Measurement time: 2.116ms
  INA260_TIME_4_156_ms, ///< Measurement time: 4.156ms
  INA260_TIME_8_244_ms, ///< Measurement time: 8.224ms
} INA260_ConversionTime;

/**
 * @brief Averaging Count options.
 *
 * Allowed values forsetAveragingCount.
 */
typedef enum _count {
  INA260_COUNT_1, ///< Window size: 1 sample (Default)
  INA260_COUNT_4, ///< Window size: 4 samples
  INA260_COUNT_16, ///< Window size: 16 samples
  INA260_COUNT_64, ///< Window size: 64 samples
  INA260_COUNT_128, ///< Window size: 128 samples
  INA260_COUNT_256, ///< Window size: 256 samples
  INA260_COUNT_512, ///< Window size: 512 samples
  INA260_COUNT_1024, ///< Window size: 1024 samples
} INA260_AveragingCount;

/**
 * Initialize the ina260 driver
 * @param i2cPort  I2C port ( a CPAL_InitTypeDef) the ina260 is connected to.
 *
 * @return True on success, else false.
 */
bool ina260Init(I2C_Dev *i2cPort);

/**
 * Enable the ina260 and configure it.
 *
 * @return True on success, else false.
 */
bool ina260SetEnabled(bool enable);

/**
 * Get measurement data.
 *
 * @return True on success, else false.
 */
bool ina260GetData(int16_t *measuredVoltage);

#endif // INA260_H
