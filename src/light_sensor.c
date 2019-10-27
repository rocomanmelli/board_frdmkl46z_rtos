/*
 * Copyright 2019, Roman Comelli
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file light_sensor.c
 * \brief Light sensor task source file
 * \details This file implements all the things needed for the light sensor
 * task.
 * \author Roman Comelli
 * \date 2019
 * \copyright 3-Clause BSD License
 * \addtogroup LightSensorTask Light sensor task
 * @{
 */


/*================================[inclusions]================================*/
/* FreeRTOS kernel headers. */
#include "FreeRTOS.h"
#include "timers.h"

/* Modules headers. */
#include "board_dsi.h"
#include "adc.h"
#include "uart_rtos.h"

/* Standard C headers. */
#include <stdio.h>
#include <string.h>


/*==========================[macros and definitions]==========================*/
#define SAMPLES_COUNT 20
#define LENGTH_STR_BUFFER 20
#define LUZ_THR 2000
#define TURNED_ON_TIME 50
#define TURNED_OFF_TIME 100


/*=================================[typedefs]=================================*/
typedef enum{
    TURNED_OFF = 0,
    TURNED_ON_WAITING,
    TURNED_ON,
    TURNED_OFF_WAITING,
} LightSensorStates;


/*=====================[internal functions declarations]=====================*/
static void LightSensorTask(TimerHandle_t xTimer);


/*=========================[internal data definition]=========================*/
/**
 * \var static uint32_t $timestamp
 * Static variable used to generate time stamps.
 */
static uint32_t timestamp = 0;


/*=========================[external data definition]=========================*/


/*======================[internal functions definitions]======================*/
/**
 * \brief Controls everything related to the light sensor task.
 * \details Triggers the ADC to measure the output of the light sensor and,
 * depending on the acquired value, its finite state machine send a message
 * through the UART.
 * @param[in]  xTimer  Handle for a timer object (required by the FreeRTOS API).
 */
static void LightSensorTask(TimerHandle_t xTimer){
    (void) xTimer;
    int32_t light_average = 0;
    static uint8_t samples = 0, index = 0, ms_count = 0;
    static LightSensorStates state = TURNED_OFF;
    uint8_t i;
    char str[LENGTH_STR_BUFFER];
    static int32_t light_measurement[SAMPLES_COUNT];

    timestamp++;

    /* Getting value. */
    ADC_IniciarConv();
    adc_getValueBlocking(light_measurement + index, 1);
    /* Execution never gets to this point without a measurement. */
    if (++index == SAMPLES_COUNT){
        index = 0;
    }

    /* Processing (till sample 20, average is not representative). */
    if (samples < SAMPLES_COUNT - 1){
        samples++;
    }
    else{
        /* Calculating average. */
        for (i = 0; i < SAMPLES_COUNT; i++){
            light_average += light_measurement[i];
        }
        light_average /= SAMPLES_COUNT;
        /* Finite state machine. */
        switch (state){
            case TURNED_OFF:
                if (light_average < LUZ_THR){
                    board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
                    snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:ON\r\n", timestamp);
                    (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                    state = TURNED_ON_WAITING;
                }
                break;
            case TURNED_ON_WAITING:
                ms_count++;
                if (ms_count == TURNED_ON_TIME){
                    ms_count = 0;
                    if (light_average > LUZ_THR){
                        board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
                        snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:OFF\r\n", timestamp);
                        (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                        state = TURNED_OFF_WAITING;
                    }
                    else{
                        state = TURNED_ON;
                    }
                }
                break;
            case TURNED_ON:
                if (light_average > LUZ_THR){
                    board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
                    snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:OFF\r\n", timestamp);
                    (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                    state = TURNED_OFF_WAITING;
                }
                break;
            case TURNED_OFF_WAITING:
                ms_count++;
                if (ms_count == TURNED_OFF_TIME){
                    ms_count = 0;
                    if (light_average < LUZ_THR){
                        board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
                        snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:ON\r\n", timestamp);
                        (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                        state = TURNED_ON_WAITING;
                    }
                    else{
                        state = TURNED_OFF;
                    }
                }
                break;
            default:
                break;
        }
    }
}


/*======================[external functions definitions]======================*/
/**
 * \brief Initializes the light sensor.
 * \details Creates a periodic task to run the light sensor task.
 */
void LightSensorInit(void){
    TimerHandle_t periodic_task_handle;

    periodic_task_handle = xTimerCreate("LightSensorTask",
                                        1 / portTICK_PERIOD_MS,
			                            pdTRUE,
		                                NULL,
			                            LightSensorTask);

    xTimerStart(periodic_task_handle, portMAX_DELAY);
}

/**
 * \brief Returns time.
 * \return The current time stamp.
 */
uint32_t GetTimeStamp(void){
    return timestamp;
}


/** @} */
