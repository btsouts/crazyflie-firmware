/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * mem.c: Memory module. Handles one-wire and eeprom memory functions over crtp link.
 */

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "config.h"

#include "console.h"
#include "assert.h"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "crtp.h"
#include "motors.h"
#include "stabilizer.h"

// Maximum log payload length
#define MEM_MAX_LEN 30
#define MAX_THRUST2  60000
#define STATUS_OK 0

typedef enum {
  TYPE_STOP_MOTORS = 0,
  TYPE_SET_EACH_MOTOR = 1,
  TYPE_SET_ALL_MOTORS = 2,
  TYPE_SET_SWEEP = 3
} pwmCRTPHeaders;

//Private functions
static void pwmCRTPTask(void * prm);

static bool isInit = false;
static CRTPPacket p;
static uint16_t ratioM1=0, ratioM2=0, ratioM3=0, ratioM4=0;

void pwmCRTPInit(void)
{
  if(isInit)
    return;

  /*
  if (owScan(&nbrOwMems))
    isInit = true;
  else
    isInit = false;
  */
  isInit = true;

  DEBUG_PRINT("Create pwmCRTPTask\n");
  //Start the mem task
  xTaskCreate(pwmCRTPTask, PWM_CRTP_TASK_NAME,
              PWM_CRTP_TASK_STACKSIZE, NULL, PWM_CRTP_TASK_PRI, NULL);
}

bool pwmCRTPTest(void)
{
  return isInit;
}

void pwmCRTPTask(void * param)
{
	crtpInitTaskQueue(CRTP_PORT_PWM);
  //static bool executedOnce = false;
  //int count=0, countMax = 10000;
  int profileSeconds=0;
  uint16_t ratioM = 0;
  uint16_t ratioMPerc=0, startPercM=0, endPercM=0, ratioMInc=0;
  
	while(1)
	{
		crtpReceivePacketBlock(CRTP_PORT_PWM, &p);

    if (p.channel == TYPE_SET_EACH_MOTOR) {
      memcpy(&ratioM1,&p.data[0],2);
      memcpy(&ratioM2,&p.data[2],2);
      memcpy(&ratioM3,&p.data[4],2);
      memcpy(&ratioM4,&p.data[6],2);
      memcpy(&profileSeconds,&p.data[8],4);
    } else if (p.channel == TYPE_SET_ALL_MOTORS) {
      memcpy(&ratioM,&p.data[0],2); 
      memcpy(&profileSeconds,&p.data[2],4);

#ifdef DEBUG
      DEBUG_PRINT("ratioM %d, profileSeconds %d\n",ratioM,profileSeconds);
#endif 

      stabilizerThrustExperiment(ratioM);

      ratioM1 = ratioM;
      ratioM2 = ratioM;
      ratioM3 = ratioM;
      ratioM4 = ratioM;

      if (ratioM > 0) {
        vTaskDelay(M2T(profileSeconds*1000));
      }
    } else if (p.channel == TYPE_STOP_MOTORS) {
    } else if (p.channel == TYPE_SET_SWEEP) {
      memcpy(&startPercM,&p.data[0],2);
      memcpy(&endPercM,&p.data[2],2);
      memcpy(&ratioMInc,&p.data[4],2);  
      memcpy(&profileSeconds,&p.data[6],4);


#ifdef DEBUG
      DEBUG_PRINT("startPercM %d, endPercM %d, ratioMInc %d, profileSeconds %d\n",startPercM, endPercM, ratioMInc, profileSeconds);
#endif 

      for (ratioMPerc=startPercM; ratioMPerc <= endPercM; ratioMPerc += ratioMInc) {
        ratioM = ratioMPerc * MAX_THRUST2 / 100; //UINT16_MAX

        stabilizerThrustExperiment(ratioM);

        ratioM1 = ratioM;
        ratioM2 = ratioM;
        ratioM3 = ratioM;
        ratioM4 = ratioM;

        if (ratioM > 0) {
          vTaskDelay(M2T(profileSeconds*1000));
        }
      }

    } else {
      DEBUG_PRINT("Unknown channel value %d\n",p.channel);    
    }


	}
}

/*
PARAM_GROUP_START(memTst)
  PARAM_ADD(PARAM_UINT8, resetW, &memTesterWriteReset)
PARAM_GROUP_STOP(memTst)
 */

LOG_GROUP_START(pwmCRTP)
  LOG_ADD(LOG_UINT16, ratioM1, &ratioM1)
  LOG_ADD(LOG_UINT16, ratioM2, &ratioM2)
  LOG_ADD(LOG_UINT16, ratioM3, &ratioM3)
  LOG_ADD(LOG_UINT16, ratioM4, &ratioM4)
LOG_GROUP_STOP(pwmCRTP)
