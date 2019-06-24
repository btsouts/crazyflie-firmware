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
 * @file ina219.c
 * Driver for the ina219 XXX.
 *
 */
#define DEBUG_MODULE "INA219"

#include "FreeRTOS.h"
#include "task.h"

#include "ina219.h"
#include "i2cdev.h"
#include "debug.h"
#include "eprintf.h"
#include "param.h"
#include "log.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

static int16_t loadVoltage=0, shuntVoltage=0;
static int16_t loadCurrent=0;
static uint16_t ina219_currentDivider_mA; 

static void ina219Task(void *param);

bool ina219Init(I2C_Dev *i2cPort)
{
  if (isInit)
    return true;

  I2Cx = i2cPort;
  devAddr = INA219_ADDRESS;

  vTaskDelay(M2T(5));

  isInit = true;

  //DEBUG_PRINT("Init INA 219\n");
  //ina219Init(I2C1_DEV);

  DEBUG_PRINT("Enable INA 219\n");
  ina219SetEnabled(true);

  xTaskCreate(ina219Task, INA219_TASK_NAME,
               INA219_TASK_STACKSIZE, NULL, INA219_TASK_PRI, NULL);

  return true;
}

bool ina219SetEnabled(bool enable)
{
  bool status=false;
  uint16_t ina219_calValue = 8192; //setCalibration_16V_400mA
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
					          INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  uint8_t data[2];

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
  //ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

	if (!isInit)
	  return false;

	if (enable)
	{
	  data[0] = (uint8_t) (ina219_calValue & 0xFF00) >> 8; //MSB
    data[1] = (uint8_t) (ina219_calValue & 0x00FF);      //LSB

    DEBUG_PRINT("Writing calibration\n");
    status = i2cdevWriteReg8(I2Cx, devAddr, INA219_REG_CALIBRATION, 2, data);
    //DEBUG_PRINT("Status is %d\n",status);

    data[0] = (uint8_t) (config & 0xFF00) >> 8; //MSB
    data[1] = (uint8_t) (config & 0x00FF);      //LSB

    DEBUG_PRINT("Writing config\n");
    status = i2cdevWriteReg8(I2Cx, devAddr, INA219_REG_CONFIG, 2, data);
    //DEBUG_PRINT("Status is %d\n",status);
	}
	else
	{
	  // Power off and default values
	}

	return status;
}

bool ina219GetData(int16_t *measuredValue)
{
  uint8_t data[2];
  bool status;

  uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;

#ifdef MEASURE_VOLTAGE
  int16_t		busVoltage=0; //, loadVoltage=0;

  status =  i2cdevReadReg8(I2Cx, devAddr, INA219_REG_BUSVOLTAGE, 2, data);
  readSensorRegisterValueMSB = data[0];
	readSensorRegisterValueLSB = data[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);
  //DEBUG_PRINT("1Raw dats is MSB 0x%X LSB 0x%X \n",readSensorRegisterValueMSB,readSensorRegisterValueMSB);
  readSensorRegisterValueCombined = (readSensorRegisterValueCombined >> 3) * 4; // Enable this for bus voltage
	busVoltage = (int16_t) readSensorRegisterValueCombined;

  status =  i2cdevReadReg8(I2Cx, devAddr, INA219_REG_SHUNTVOLTAGE, 2, data);
  readSensorRegisterValueMSB = data[0];
	readSensorRegisterValueLSB = data[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);
  shuntVoltage = (int16_t) readSensorRegisterValueCombined;

	//loadVoltage = busVoltage + (shuntVoltage / 1000);
  loadVoltage = busVoltage + shuntVoltage;

  *measuredValue = loadVoltage;
#endif

#ifdef MEASURE_CURRENT
  // From the Adafruit library
  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  //wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

  // Now we can safely read the CURRENT register!
  status =  i2cdevReadReg8(I2Cx, devAddr, INA219_REG_CURRENT, 2, data);
  readSensorRegisterValueMSB = data[0];
	readSensorRegisterValueLSB = data[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);

  //readSensorRegisterValueCombined /= ina219_currentDivider_mA;
  loadCurrent = readSensorRegisterValueCombined;

  *measuredValue = loadCurrent;
#endif

  return status;
}

static void ina219Task(void *param)
{
  int16_t measuredValue;

  while (1)
  {
    vTaskDelay(M2T(100));

    //DEBUG_PRINT("Measure INA 219\n");
    ina219GetData(&measuredValue);

  // #ifdef MEASURE_VOLTAGE
  //       DEBUG_PRINT("Measured Voltage is %d\n",measuredValue);
  // #endif      

  // #ifdef MEASURE_CURRENT
  //       DEBUG_PRINT("Measured Current is %d\n",measuredValue);
  // #endif
  }  
}

// PARAM_GROUP_START(current_sensor)
// PARAM_ADD(PARAM_INT16 | PARAM_RONLY, INA219, &loadVoltage)
// //PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) 
// PARAM_GROUP_STOP(current_sensor)

LOG_GROUP_START(current_sensor)
LOG_ADD(LOG_INT16, ina219LVT, &loadVoltage)
LOG_ADD(LOG_INT16, ina219SVT, &shuntVoltage)
LOG_ADD(LOG_INT16, ina219CUR, &loadCurrent)
LOG_GROUP_STOP(current_sensor)