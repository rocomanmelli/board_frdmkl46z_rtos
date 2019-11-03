/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/
#include <modules/board_frdmkl46z_rtos/inc/mma8451.h>
#include "fsl_i2c.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "uart_rtos.h"
#include "mma8451.h"
#include <stdio.h>
#include "stdbool.h"


/**
 * \file acc.c
 * \brief Accelerometer user application task header file
 * \details This file implements all the things needed for the accelerometer
 * task.
 * \author Roman Comelli
 * \date 2019
 * \copyright 3-Clause BSD License
 * \addtogroup UserTaskAcc Accelerometer task
 * @{
 */


/*==========================[macros and definitions]==========================*/
#define ACC_THR 100
#define ACC_CNT 10
#define LENGTH_STR_BUFFER 40


/*=================================[typedefs]=================================*/
typedef enum{
    TRANSMITTING_INIT = 0,
    TRANSMITTING_OFF,
	TRANSMITTING_ON,
} AccelerometerStates;


/*=====================[internal functions declarations]=====================*/
static void UserTaskAcc(void *pvParameters);


/*=========================[internal data definition]=========================*/


/*=========================[external data definition]=========================*/


/*======================[internal functions definitions]======================*/
/**
 * \brief Controls everything related to the accelerometer (user application)
 * task.
 * \details Reads what was written in the queue of the accelerometer service and
 * depending on the acquired value, controls the finite state machine send a
 * message through the UART.
 * @param[in]  pvParameters  Not used (required by the FreeRTOS API).
 */
static void UserTaskAcc(void *pvParameters)
{
	(void) pvParameters;
	static AccelerometerStates state = 0;
	char str[LENGTH_STR_BUFFER];
	uint8_t sent_count = 0;
	TickType_t timestamp;
    accelerations_t accelerations;
    static int16_t readX_old = 0;
    static int16_t readY_old = 0;
    static int16_t readZ_old = 0;

	while (1)
	{
        if (acc_getValueBlocking(&accelerations, portMAX_DELAY)){
            /* Get the time the function started. */
            timestamp = xTaskGetTickCount();

            switch (state){
                case TRANSMITTING_INIT: /* To avoid first data to be printed. */
                    state = TRANSMITTING_OFF;
                    break;
                case TRANSMITTING_OFF:
                    if (accelerations.readX < readX_old - ACC_THR || accelerations.readX > readX_old + ACC_THR
                    || accelerations.readY < readY_old - ACC_THR || accelerations.readY > readY_old + ACC_THR
                    || accelerations.readZ < readZ_old - ACC_THR || accelerations.readZ > readZ_old + ACC_THR){
                        snprintf(str, LENGTH_STR_BUFFER, "[%d] ACC:X=%d;Y=%d;Z=%d\r\n", timestamp, accelerations.readX, accelerations.readY, accelerations.readZ);
                        (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), portMAX_DELAY);
                        sent_count = 0;
                        state = TRANSMITTING_ON;
                    }
                    break;
                case TRANSMITTING_ON:
                    sent_count++;
                    snprintf(str, LENGTH_STR_BUFFER, "[%d] ACC:X=%d;Y=%d;Z=%d\r\n", timestamp, accelerations.readX, accelerations.readY, accelerations.readZ);
                    (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), portMAX_DELAY);
                    if (sent_count >= ACC_CNT - 1){
                        state = TRANSMITTING_OFF;
                    }
                    break;
                default:
                    break;
            }

            readX_old = accelerations.readX;
            readY_old = accelerations.readY;
            readZ_old = accelerations.readZ;
        }
	}
}

/*==================[external functions definition]==========================*/
/**
 * \brief Creates the task for the accelerometer (user application).
 */
void UserAccInit(void)
{
    xTaskCreate(UserTaskAcc, "UserTaskAcc", 200, NULL, 1, NULL);
}

/*==================[end of file]============================================*/
