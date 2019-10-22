/* Copyright 2019, DSI FCEIA UNR
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2019, Gustavo Muro
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

// Standard C Included Files
#include <board_dsi.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// Project Included Files
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_port.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/*==================[macros and definitions]=================================*/
#define DEMO_ADC16_CHANNEL_GROUP 	0U
#define DEMO_ADC16_USER_CHANNEL 	3U /* PTE22*/

#define SENSOR_LUZ_PORT         	PORTE
#define SENSOR_LUZ_PIN          	22

/*==================[internal data declaration]==============================*/
static adc16_channel_config_t adc16_channel;
static volatile uint32_t g_Adc16ConversionValue;
static volatile uint32_t g_Adc16InterruptCounter;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
static int32_t acumLectAdc;
static int32_t contLectAdc;

static QueueHandle_t xADCQueue;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void ADC_config(void)
{
    adc16_config_t adc16_config;

    PORT_SetPinMux(SENSOR_LUZ_PORT, SENSOR_LUZ_PIN, kPORT_PinDisabledOrAnalog);

	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */

    ADC16_GetDefaultConfig(&adc16_config);
	ADC16_Init(ADC0, &adc16_config);
	ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */

	adc16_channel.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16_channel.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
	adc16_channel.enableDifferentialConversion = false;

	g_Adc16InterruptCounter = 0U;

	NVIC_EnableIRQ(ADC0_IRQn);
}

void ADC_IniciarConv(void)
{
	/*
	 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
	 function, which works like writing a conversion command and executing it. For another channel's conversion,
	 just to change the "channelNumber" field in channel configuration structure, and call the function
	 "ADC16_ChannelConfigure()"" again.
	 Also, the "enableInterruptOnConversionCompleted" inside the channel configuration structure is a parameter for
	 the conversion command. It takes affect just for the current conversion. If the interrupt is still required
	 for the following conversion, it is necessary to assert the "enableInterruptOnConversionCompleted" every time
	 for each command.
	*/
	ADC16_SetChannelConfig(ADC0, DEMO_ADC16_CHANNEL_GROUP, &adc16_channel);
}

void vCallbackFunction( TimerHandle_t xTimer )
{
	(void) xTimer; /* To avoid warning. */
	ADC_IniciarConv();
}

/*==================[external functions definition]==========================*/

void adc_init(int32_t sampleTime)
{
	TimerHandle_t xTimer;

    xADCQueue = xQueueCreate( 10, sizeof( int32_t ) );

#ifdef DEBUG
    vQueueAddToRegistry( xADCQueue, "adcQ" );
#endif

	// Se inicializa el ADC
	ADC_config();

	if (sampleTime > 0){
		xTimer = xTimerCreate("trigerADC",
    		sampleTime / portTICK_PERIOD_MS,
			pdTRUE,
			NULL,
			vCallbackFunction);
		xTimerStart(xTimer, portMAX_DELAY);
	}
}

int32_t adc_getVal(void)
{
	int32_t ret = 0, divisor, dividendo;

	NVIC_DisableIRQ(ADC0_IRQn);
	divisor = contLectAdc;
	dividendo = acumLectAdc;
	acumLectAdc = 0;
	contLectAdc = 0;
	NVIC_EnableIRQ(ADC0_IRQn);

	if (divisor != 0)
		ret = dividendo / divisor;

	return ret;
}

bool adc_getValueBlocking(int32_t *lect, int32_t timeToWait)
{
	if (xQueueReceive(xADCQueue, lect, timeToWait / portTICK_PERIOD_MS) == pdTRUE)
		return true;
	else
		return false;
}

void ADC0_IRQHandler(void)
{
	int32_t adcLect;

    /* Read conversion result to clear the conversion completed flag. */
	adcLect = ADC16_GetChannelConversionValue(ADC0, DEMO_ADC16_CHANNEL_GROUP);

	/* promedio automático */
	acumLectAdc += adcLect;
	contLectAdc++;

    /* almacenamiento en queue */
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/* coloco promedio en queue */
	xQueueSendFromISR( xADCQueue, &adcLect, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*==================[end of file]============================================*/
