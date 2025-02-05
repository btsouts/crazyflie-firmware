/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 * worker.c - Worker system that can execute asynchronous actions in tasks
 */
#include "worker.h"

#include <errno.h>

#include <limits.h>
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"

//#include "ina219.h"
//#include "i2cdev.h"

#include "console.h"

#define WORKER_QUEUE_LENGTH 5

struct worker_work {
  void (*function)(void*);
  void* arg;
};

static xQueueHandle workerQueue;

void workerInit()
{
  if (workerQueue)
    return;

  workerQueue = xQueueCreate(WORKER_QUEUE_LENGTH, sizeof(struct worker_work));
  DEBUG_QUEUE_MONITOR_REGISTER(workerQueue);
}

bool workerTest()
{
  return (workerQueue != NULL);
}

void workerLoop()
{
  struct worker_work work;
  // unsigned int printCounter = 0, printCounts=0;
  // int16_t measuredValue;

  if (!workerQueue)
    return;

  // DEBUG_PRINT("Init INA 219\n");
  // ina219Init(I2C1_DEV);

  // DEBUG_PRINT("Enable INA 219\n");
  // ina219SetEnabled(true);

  while (1)
  {
    xQueueReceive(workerQueue, &work, portMAX_DELAY);
    
    if (work.function)
      work.function(work.arg);

//     if (printCounter == 100) {
//       printCounter = 0;
//       printCounts++;

//       DEBUG_PRINT("Measure INA 219\n");
//       ina219GetData(&measuredValue);

// // #ifdef MEASURE_VOLTAGE
// //       DEBUG_PRINT("Measured Voltage is %d\n",measuredValue);
// // #endif      

// #ifdef MEASURE_CURRENT
//       DEBUG_PRINT("Measured Current is %d\n",measuredValue);
// #endif
    // } else {
    //   printCounter++;
    // }
    
  }
}

int workerSchedule(void (*function)(void*), void *arg)
{
  struct worker_work work;
  
  if (!function)
    return ENOEXEC;
  
  work.function = function;
  work.arg = arg;
  if (xQueueSend(workerQueue, &work, 0) == pdFALSE)
    return ENOMEM;

  return 0; 
}

