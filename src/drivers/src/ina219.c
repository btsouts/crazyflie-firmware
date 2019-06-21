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

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

bool ina219Init(I2C_Dev *i2cPort)
{
  if (isInit)
    return true;

  I2Cx = i2cPort;
  devAddr = INA219_ADDRESS;

  vTaskDelay(M2T(5));

  isInit = true;

  return true;
}

/*
bool ina219TestConnection(void)
{
  uint8_t id;
  bool status;

  if (!isInit)
	return false;


  status = i2cdevReadReg8(I2Cx, devAddr, LPS25H_WHO_AM_I, 1, &id);

  if (status == true && id == LPS25H_WAI_ID)
	  return true;

  return false;
}

bool ina219SelfTest(void)
{
  float pressure;
  float temperature;
  float asl;

  if (!isInit)
    return false;

  ina219GetData(&pressure, &temperature, &asl);

  if (ina219EvaluateSelfTest(LPS25H_ST_PRESS_MIN, LPS25H_ST_PRESS_MAX, pressure, "pressure") &&
      ina219EvaluateSelfTest(LPS25H_ST_TEMP_MIN, LPS25H_ST_TEMP_MAX, temperature, "temperature"))
  {
    return true;
  }
  else
  {
   return false;
  }
}

bool ina219EvaluateSelfTest(float min, float max, float value, char* string)
{
  if (value < min || value > max)
  {
    DEBUG_PRINT("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, (double)min, (double)max, (double)value);
    return false;
  }
  return true;
}
*/

bool ina219SetEnabled(bool enable)
{
  bool status;
  uint16_t ina219_calValue = 8192; //setCalibration_16V_400mA
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
					          INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  uint8_t data[2];

	if (!isInit)
	  return false;

	if (enable)
	{
	  data[0] = (uint8_t) (ina219_calValue & 0xFF00) >> 8; //MSB
    data[1] = (uint8_t) (ina219_calValue & 0x00FF);      //LSB

    DEBUG_PRINT("Writing calibration\n");
    status = i2cdevWriteReg8(I2Cx, devAddr, INA219_REG_CALIBRATION, 2, data);
    DEBUG_PRINT("Status is %d\n",status);

    data[0] = (uint8_t) (config & 0xFF00) >> 8; //MSB
    data[1] = (uint8_t) (config & 0x00FF);      //LSB

    DEBUG_PRINT("Writing config\n");
    status = i2cdevWriteReg8(I2Cx, devAddr, INA219_REG_CONFIG, 2, data);
    DEBUG_PRINT("Status is %d\n",status);
	}
	else
	{
	  // Power off and default values
	}

	return status;
}

bool ina219GetData(int16_t *measuredVoltage)
{
  uint8_t data[2];
  bool status;

  uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int16_t		busVoltage=0, shuntVoltage=0, loadVoltage=0;

  status =  i2cdevReadReg8(I2Cx, devAddr, INA219_REG_BUSVOLTAGE, 2, data);
  //DEBUG_PRINT("Status is %d\n",status);

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

	loadVoltage = busVoltage + (shuntVoltage / 1000);

  *measuredVoltage = loadVoltage;

  return status;
}

//#include "math.h"

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float ina219PressureToAltitude(float* pressure)
{
    return 0;
}
