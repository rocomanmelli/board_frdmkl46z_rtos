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
#include "semphr.h"
#include "task.h"
#include "uart_rtos.h"
#include "light_sensor.h"
#include <stdio.h>


// COMMENT
/*==================[macros and definitions]=================================*/
#define MMA8451_I2C_ADDRESS     (0x1d)

#define INT1_PORT       PORTC
#define INT1_GPIO       GPIOC
#define INT1_PIN        5

#define ACC_THR 100
#define ACC_CNT 10
#define LENGTH_STR_BUFFER 40


typedef enum{
    TRANSMITTING_OFF = 0,
	TRANSMITTING_ON,
} AccelerometerStates;

typedef union
{
    struct
    {
        unsigned SRC_DRDY:1;
        unsigned :1;
        unsigned SRC_FF_MT:1;
        unsigned SRC_PULSE:1;
        unsigned SRC_LNDPRT:1;
        unsigned SRC_TRANS:1;
        unsigned SRC_FIFO:1;
        unsigned SRC_ASLP:1;
    };
    uint8_t data;
}INT_SOURCE_t;

typedef union
{
    struct
    {
        unsigned XDR:1;
        unsigned YDR:1;
        unsigned ZDR:1;
        unsigned ZYXDR:1;
        unsigned XOW:1;
        unsigned YOW:1;
        unsigned ZOW:1;
        unsigned ZYXOW:1;
    };
    uint8_t data;
}STATUS_t;

typedef union
{
    struct
    {
        unsigned ACTIVE:1;
        unsigned F_READ:1;
        unsigned LNOISE:1;
        unsigned DR:3;
        unsigned ASLP_RATE:2;
    };
    uint8_t data;
}CTRL_REG1_t;

#define CTRL_REG1_ADDRESS   0X2A

typedef union
{
    struct
    {
        unsigned INT_EN_DRDY:1;
        unsigned :1;
        unsigned INT_EN_FF_MT:1;
        unsigned INT_EN_PULSE:1;
        unsigned INT_EN_LNDPRT:1;
        unsigned INT_EN_TRANS:1;
        unsigned INT_EN_FIFO:1;
        unsigned INT_EN_ASLP:1;
    };
    uint8_t data;
}CTRL_REG4_t;

#define CTRL_REG4_ADDRESS   0X2D

typedef union
{
    struct
    {
        unsigned INT_CFG_DRDY:1;
        unsigned :1;
        unsigned INT_CFG_FF_MT:1;
        unsigned INT_CFG_PULSE:1;
        unsigned INT_CFG_LNDPRT:1;
        unsigned INT_CFG_TRANS:1;
        unsigned INT_CFG_FIFO:1;
        unsigned INT_CFG_ASLP:1;
    };
    uint8_t data;
}CTRL_REG5_t;

#define CTRL_REG5_ADDRESS   0X2E

#define INT_SOURCE_ADDRESS   0X0C
#define STATUS_ADDRESS       0X00

/*==================[internal data declaration]==============================*/

static int16_t readX = 0;
static int16_t readY = 0;
static int16_t readZ = 0;
static int16_t readX_old = 0;
static int16_t readY_old = 0;
static int16_t readZ_old = 0;

/*==================[internal functions declaration]=========================*/
static uint8_t mma8451_read_reg(uint8_t addr)
{
	i2c_master_transfer_t masterXfer;
    uint8_t ret;

	memset(&masterXfer, 0, sizeof(masterXfer));
	masterXfer.slaveAddress = MMA8451_I2C_ADDRESS;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &ret;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &masterXfer);

	return ret;
}

static void mma8451_write_reg(uint8_t addr, uint8_t data)
{
	i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = MMA8451_I2C_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &data;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(I2C0, &masterXfer);
}

/*==================[internal data definition]===============================*/

static SemaphoreHandle_t xSemINTAcc;
static SemaphoreHandle_t xMutexAcc;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void config_port_int1(void)
{
	const port_pin_config_t port_int1_config = {
			/* Internal pull-up/down resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Slow slew rate is configured */
		.slewRate = kPORT_SlowSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};
	const gpio_pin_config_t gpio_int1_config = {
		.pinDirection = kGPIO_DigitalInput,
		.outputLogic = 0U
	};

	PORT_SetPinConfig(INT1_PORT, INT1_PIN, &port_int1_config);
	GPIO_PinInit(INT1_GPIO, INT1_PIN, &gpio_int1_config);

	/* Interrupt polarity active high, or active low. Default value: 0.
	   0: Active low; 1: Active high. VER REGISTRO CTRL_REG3 */
	PORT_SetPinInterruptConfig(INT1_PORT, INT1_PIN, kPORT_InterruptLogicZero);

	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	NVIC_SetPriority(PORTC_PORTD_IRQn, 0);
}

static void taskAcc(void *pvParameters)
{
	(void) pvParameters;
    int16_t readG;
    INT_SOURCE_t intSource;
    STATUS_t status;
	static AccelerometerStates state = 0;
	char str[LENGTH_STR_BUFFER];
	uint8_t sent_count = 0;
	TickType_t timestamp;

	while (1)
	{
		xSemaphoreTake(xSemINTAcc, portMAX_DELAY);

		xSemaphoreTake(xMutexAcc, portMAX_DELAY);

	    intSource.data = mma8451_read_reg(INT_SOURCE_ADDRESS);

	    if (intSource.SRC_DRDY)
	    {
	    	status.data = mma8451_read_reg(STATUS_ADDRESS);

	        if (status.XDR)
	        {
				readX_old = readX;
	            readG   = (int16_t)mma8451_read_reg(0x01)<<8;
	            readG  |= mma8451_read_reg(0x02);
	            readX = readG >> 2;
				readX = (int16_t) ((((int32_t) readX) * 100) / 4096);
	        }

	        if (status.YDR)
	        {
				readY_old = readY;
	            readG   = (int16_t)mma8451_read_reg(0x03)<<8;
	            readG  |= mma8451_read_reg(0x04);
	            readY = readG >> 2;
				readY = (int16_t) ((((int32_t) readY) * 100) / 4096);
	        }

	        if (status.ZDR)
	        {
				readZ_old = readZ;
	            readG   = (int16_t)mma8451_read_reg(0x05)<<8;
	            readG  |= mma8451_read_reg(0x06);
	            readZ = readG >> 2;
				readZ = (int16_t) ((((int32_t) readZ) * 100) / 4096);
	        }
	    }

		/* Get the time the function started. */
		timestamp = xTaskGetTickCount();

		switch (state){
            case TRANSMITTING_OFF:
                if (readX < readX_old - ACC_THR || readX > readX_old + ACC_THR
				|| readY < readY_old - ACC_THR || readY > readY_old + ACC_THR
				|| readZ < readZ_old - ACC_THR || readZ > readZ_old + ACC_THR){
                    snprintf(str, LENGTH_STR_BUFFER, "[%d] ACC:X=%d;Y=%d;Z=%d\r\n", timestamp, readX, readY, readZ);
                    (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), portMAX_DELAY);
					sent_count = 0;
                    state = TRANSMITTING_ON;
                }
                break;
            case TRANSMITTING_ON:
                sent_count++;
				snprintf(str, LENGTH_STR_BUFFER, "[%d] ACC:X=%d;Y=%d;Z=%d\r\n", timestamp, readX, readY, readZ);
                (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), portMAX_DELAY);
                if (sent_count >= ACC_CNT - 1){
                    state = TRANSMITTING_OFF;
                }
                break;
            default:
                break;
        }



	    xSemaphoreGive(xMutexAcc);

	    PORT_SetPinInterruptConfig(INT1_PORT, INT1_PIN, kPORT_InterruptLogicZero);
	}
}

/*==================[external functions definition]==========================*/
void mma8451_init(void)
{
    CTRL_REG1_t ctrl_reg1;
    CTRL_REG4_t ctrl_reg4;
    CTRL_REG5_t ctrl_reg5;

    xMutexAcc = xSemaphoreCreateMutex();

    xSemaphoreTake(xMutexAcc, 0);

	ctrl_reg4.INT_EN_DRDY = 1;
	ctrl_reg4.INT_EN_FF_MT = 0;
	ctrl_reg4.INT_EN_PULSE = 0;
	ctrl_reg4.INT_EN_LNDPRT = 0;
	ctrl_reg4.INT_EN_TRANS = 0;
	ctrl_reg4.INT_EN_FIFO = 0;
	ctrl_reg4.INT_EN_ASLP = 0;

	mma8451_write_reg(CTRL_REG4_ADDRESS, ctrl_reg4.data);

	/* verificación */
	ctrl_reg4.data = mma8451_read_reg(CTRL_REG4_ADDRESS);

	ctrl_reg5.INT_CFG_DRDY = 1;
	ctrl_reg5.INT_CFG_FF_MT = 0;
	ctrl_reg5.INT_CFG_PULSE = 0;
	ctrl_reg5.INT_CFG_LNDPRT = 0;
	ctrl_reg5.INT_CFG_TRANS = 0;
	ctrl_reg5.INT_CFG_FIFO = 0;
	ctrl_reg5.INT_CFG_ASLP = 0;

	mma8451_write_reg(CTRL_REG5_ADDRESS, ctrl_reg5.data);

	/* verificación */
	ctrl_reg5.data = mma8451_read_reg(CTRL_REG5_ADDRESS);

	ctrl_reg1.ACTIVE = 1;
	ctrl_reg1.F_READ = 0;
	ctrl_reg1.LNOISE = 1;
	ctrl_reg1.DR = 0B101;
	ctrl_reg1.ASLP_RATE = 0B00;

    mma8451_write_reg(CTRL_REG1_ADDRESS, ctrl_reg1.data);

    /* verificación */
    ctrl_reg1.data = mma8451_read_reg(CTRL_REG1_ADDRESS);

    xSemaphoreGive(xMutexAcc);

    xSemINTAcc = xSemaphoreCreateBinary();

    xSemaphoreTake(xSemINTAcc, 0);

    config_port_int1();

    xTaskCreate(taskAcc, "acc", 200, NULL, 1, NULL);
}

void mma8451_setDataRate(DR_enum rate)
{
    CTRL_REG1_t ctr_reg1;
    bool estAct;

    xSemaphoreTake(xMutexAcc, portMAX_DELAY);

    /* antes de modificar data rate es necesario poner ACTIVE = 0 */
    ctr_reg1.data = mma8451_read_reg(CTRL_REG1_ADDRESS);

    /* guarda valor que tiene ACTIVE y luego pone a cero */
    estAct = ctr_reg1.ACTIVE;
    ctr_reg1.ACTIVE = 0;

	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);

	/* actualiza DR y en la misma escritura va a restaurar ACTIVE */
	ctr_reg1.DR = rate;
	ctr_reg1.ACTIVE = estAct;

	mma8451_write_reg(CTRL_REG1_ADDRESS, ctr_reg1.data);

	/* verificación */
	ctr_reg1.data = mma8451_read_reg(0x2a);

	xSemaphoreGive(xMutexAcc);
}

int16_t mma8451_getAcX(void)
{
	return (int16_t)(((int32_t)readX * 100) / (int32_t)4096);
}

void PORTC_PORTD_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( xSemINTAcc, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    PORT_SetPinInterruptConfig(INT1_PORT, INT1_PIN, kPORT_InterruptOrDMADisabled);

    PORT_ClearPinsInterruptFlags(INT1_PORT, 1<<INT1_PIN);
}

/*==================[end of file]============================================*/
