/* Copyright 2019, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2019, 2018, Gustavo Muro
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
#include "fsl_lpsci.h"
#include "fsl_port.h"
#include "board.h"
#include "MKL46Z4.h"
#include "pin_mux.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static QueueHandle_t qSend;
static QueueHandle_t qRecv;
static bool txHasEnded;

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void uart_rtos_init(void)
{
	lpsci_config_t config;

	qSend = xQueueCreate(100, sizeof(uint8_t));
	qRecv = xQueueCreate(100, sizeof(uint8_t));

#ifdef DEBUG
    vQueueAddToRegistry( qSend, "uartTxQ" );
    vQueueAddToRegistry( qRecv, "uartRxQ" );
#endif


	CLOCK_SetLpsci0Clock(0x1U);

	/* PORTA1 (pin 35) is configured as UART0_RX */
	PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);

	/* PORTA2 (pin 36) is configured as UART0_TX */
	PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);

	/*
	 * config.parityMode = kLPSCI_ParityDisabled;
	 * config.stopBitCount = kLPSCI_OneStopBit;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	LPSCI_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.parityMode = kLPSCI_ParityDisabled;
	config.stopBitCount = kLPSCI_OneStopBit;
	config.enableTx = true;
	config.enableRx = true;

	LPSCI_Init(UART0, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));

	/* Habilita interrupciones */
	LPSCI_EnableInterrupts(UART0, kLPSCI_RxDataRegFullInterruptEnable);
	LPSCI_EnableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
	NVIC_EnableIRQ(UART0_IRQn);
}

/** \brief recibe datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde guardar los datos
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes recibidos
 **/
int32_t uart_rtos_recDatos(uint8_t *pBuf, int32_t size, uint32_t blockTime)
{
    int32_t ret = 0;
    TickType_t waitTick;

    if (blockTime != portMAX_DELAY)
    	waitTick = blockTime / portTICK_PERIOD_MS;
    else
    	waitTick = portMAX_DELAY;


    /* ATENCIÓN CORREGIR ERROR */

    while ((ret < size) &&
           (xQueueReceive( qRecv, &pBuf[ret], waitTick) == pdTRUE) )
    {
    	ret++;
    	waitTick = 0;
    }

    return ret;
}

/** \brief envía datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde estan los datos a enviar
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes enviados
 **/
int32_t uart_rtos_envDatos(uint8_t *pBuf, int32_t size, uint32_t blockTime)
{
    int32_t ret = 0;
    TickType_t waitTick;

    if (blockTime != portMAX_DELAY)
    	waitTick = blockTime / portTICK_PERIOD_MS;
    else
    	waitTick = portMAX_DELAY;

    while (ret < size && xQueueSend(qSend, &pBuf[ret], waitTick) == pdTRUE)
    {
    	if (txHasEnded)
    	{
    		txHasEnded = false;
    		LPSCI_EnableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
    	}
    	ret++;
    }

    return ret;
}


void UART0_IRQHandler(void)
{
	uint8_t data;
	BaseType_t xHigherPriorityTaskWoken;

	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;

    if ( (kLPSCI_RxDataRegFullFlag)            & LPSCI_GetStatusFlags(UART0) &&
         (kLPSCI_RxDataRegFullInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0) )
	{
        /* obtiene dato recibido por puerto serie */
	    data = LPSCI_ReadByte(UART0);

		/* pone dato en queue */
	    xQueueSendFromISR( qRecv, &data, &xHigherPriorityTaskWoken );

		LPSCI_ClearStatusFlags(UART0, kLPSCI_RxDataRegFullFlag);
	}

	if ( (kLPSCI_TxDataRegEmptyFlag)            & LPSCI_GetStatusFlags(UART0) &&
         (kLPSCI_TxDataRegEmptyInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0) )
	{
		if (xQueueReceiveFromISR( qSend,(void *) &data, &xHigherPriorityTaskWoken) == pdTRUE)
		{
			/* envía dato extraído del RB al puerto serie */
			LPSCI_WriteByte(UART0, data);
		}
		else
		{
			/* si el RB está vacío deshabilita interrupción TX */
			txHasEnded = true;
			LPSCI_DisableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
		}

		LPSCI_ClearStatusFlags(UART0, kLPSCI_TxDataRegEmptyFlag);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/*==================[end of file]============================================*/

