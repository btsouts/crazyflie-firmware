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
 * @file ina260.c
 * Driver for the ina260 XXX.
 *
 */
#define DEBUG_MODULE "INA260"

#include "FreeRTOS.h"
#include "task.h"

#include "ina260.h"
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
//static uint16_t ina260_currentDivider_mA; 

static void ina260Task(void *param);

bool ina260Init(I2C_Dev *i2cPort)
{
  if (isInit)
    return true;

  I2Cx = i2cPort;
  devAddr = INA260_I2CADDR_DEFAULT;

  vTaskDelay(M2T(5));

  isInit = true;

  //DEBUG_PRINT("Init INA 260\n");
  //ina260Init(I2C1_DEV);

  DEBUG_PRINT("Enable INA 260\n");
  ina260SetEnabled(true);

  xTaskCreate(ina260Task, INA260_TASK_NAME,
               INA260_TASK_STACKSIZE, NULL, INA260_TASK_PRI, NULL);

  return true;
}

bool ina260SetEnabled(bool enable)
{
  bool status=false;

	if (!isInit)
	  return false;

	if (enable)
	{
	  DEBUG_PRINT("INA260 Set Enabled\n");
    // data[0] = (uint8_t) (ina260_calValue & 0xFF00) >> 8; //MSB
    // data[1] = (uint8_t) (ina260_calValue & 0x00FF);      //LSB

    // DEBUG_PRINT("Writing calibration\n");
    // status = i2cdevWriteReg8(I2Cx, devAddr, INA260_REG_CALIBRATION, 2, data);
    // //DEBUG_PRINT("Status is %d\n",status);

    // data[0] = (uint8_t) (config & 0xFF00) >> 8; //MSB
    // data[1] = (uint8_t) (config & 0x00FF);      //LSB

    // DEBUG_PRINT("Writing config\n");
    // status = i2cdevWriteReg8(I2Cx, devAddr, INA260_REG_CONFIG, 2, data);
    // //DEBUG_PRINT("Status is %d\n",status);
	}
	else
	{
	  // Power off and default values
	}

	return status;
}

bool ina260GetData(int16_t *measuredValue)
{
  uint8_t data[2];
  bool status;

  uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;

#ifdef MEASURE_VOLTAGE
  int16_t		busVoltage=0; //, loadVoltage=0;

  status =  i2cdevReadReg8(I2Cx, devAddr, INA260_REG_BUSVOLTAGE, 2, data);
  readSensorRegisterValueMSB = data[0];
	readSensorRegisterValueLSB = data[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);
  //DEBUG_PRINT("1Raw dats is MSB 0x%X LSB 0x%X \n",readSensorRegisterValueMSB,readSensorRegisterValueMSB);
  readSensorRegisterValueCombined = readSensorRegisterValueCombined + (readSensorRegisterValueCombined / 4);// * 1.25;
	busVoltage = (int16_t) readSensorRegisterValueCombined;
  //DEBUG_PRINT("busVoltage %d\n",busVoltage);

  // status =  i2cdevReadReg8(I2Cx, devAddr, INA260_REG_SHUNTVOLTAGE, 2, data);
  // readSensorRegisterValueMSB = data[0];
	// readSensorRegisterValueLSB = data[1];
	// readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);
  // shuntVoltage = (int16_t) readSensorRegisterValueCombined;

	// loadVoltage = busVoltage + shuntVoltage;
  loadVoltage = busVoltage;

  *measuredValue = loadVoltage;
#endif

#ifdef MEASURE_CURRENT
  status =  i2cdevReadReg8(I2Cx, devAddr, INA260_REG_CURRENT, 2, data);
  readSensorRegisterValueMSB = data[0];
	readSensorRegisterValueLSB = data[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);

  loadCurrent = readSensorRegisterValueCombined;

  *measuredValue = loadCurrent;
#endif

  return status;
}

static void ina260Task(void *param)
{
  int16_t measuredValue;

  while (1)
  {
    vTaskDelay(M2T(100));

    //DEBUG_PRINT("Measure INA 260\n");
    ina260GetData(&measuredValue);

  // #ifdef MEASURE_VOLTAGE
  //       DEBUG_PRINT("Measured Voltage is %d\n",measuredValue);
  // #endif      

  // #ifdef MEASURE_CURRENT
  //       DEBUG_PRINT("Measured Current is %d\n",measuredValue);
  // #endif
  }  
}

// PARAM_GROUP_START(current_sensor)
// PARAM_ADD(PARAM_INT16 | PARAM_RONLY, INA260, &loadVoltage)
// //PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) 
// PARAM_GROUP_STOP(current_sensor)

LOG_GROUP_START(current_sensor)
LOG_ADD(LOG_INT16, ina260LVT, &loadVoltage)
LOG_ADD(LOG_INT16, ina260SVT, &shuntVoltage)
LOG_ADD(LOG_INT16, ina260CUR, &loadCurrent)
LOG_GROUP_STOP(current_sensor)