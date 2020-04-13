/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "rs485.h"
#include "usart.h"
//#include "rtdevice.h"
//#include "bsp.h"

extern uint16_t uartTransmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
extern uint16_t uartread(uint8_t *pData, uint16_t Size, uint32_t Timeout);

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/
//ALIGN(RT_ALIGN_SIZE)
/* software simulation serial transmit IRQ handler thread stack */
/* software simulation serial transmit IRQ handler thread */
//static struct rt_thread thread_serial_soft_trans_irq;
static osThreadId thread_serial_soft_trans_irq;
/* serial event */
static EventGroupHandle_t event_serial;
/* modbus master serial device */

/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);
static int serial_rx_ind(void);
static void serial_soft_trans_irq(void const * parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{
	ucPORT = ucPORT;
	ulBaudRate = ulBaudRate;
	ucDataBits = ucDataBits;
	eParity = eParity;
	pF_UartRevIsr[POWER_UART_NUM] = serial_rx_ind;
	uart_receive_update();
    /* software initialize */
//    rt_thread_init(&thread_serial_soft_trans_irq,
//                   "master trans",
//                   serial_soft_trans_irq,
//                   RT_NULL,
//                   serial_soft_trans_irq_stack,
//                   sizeof(serial_soft_trans_irq_stack),
//                   10, 5);
//	rt_thread_startup(&thread_serial_soft_trans_irq);
	//    rt_event_init(&event_serial, "master event", RT_IPC_FLAG_PRIO);
	osThreadDef(powerCtrTask, serial_soft_trans_irq, osPriorityNormal, 0, 1024);
	thread_serial_soft_trans_irq = osThreadCreate(osThread(powerCtrTask), NULL);
	
	event_serial = xEventGroupCreate();
	if(event_serial != NULL)
		return TRUE;
	else
		return FALSE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    EventBits_t recved_event;
	//printf("\r\nvMBMasterPortSerialEnable");
    if (xRxEnable)
    {
		//printf("\r\nxRxEnable");
        /* enable RX interrupt */
//        serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        /* switch 485 to receive mode */
        RS485_RX_EN();
    }
    else
    {
		//printf("\r\nxTxEnable");
        /* switch 485 to transmit mode */
        //rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_HIGH);
		RS485_TX_EN();
        /* disable RX interrupt */
//        serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }
    if (xTxEnable)
    {
        /* start serial transmit */
//        rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
//			printf("\r\nxEventGroupSetBits EVENT_SERIAL_TRANS_START");
			xEventGroupSetBits(event_serial, EVENT_SERIAL_TRANS_START);
    }
    else
    {
        /* stop serial transmit */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
//                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
//                &recved_event);
		  recved_event = xEventGroupWaitBits(event_serial, EVENT_SERIAL_TRANS_START,
            pdTRUE, pdTRUE, 0);
    }
}

void vMBMasterPortClose(void)
{
//    serial->parent.close(&(serial->parent));
	
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
//    serial->parent.write(&(serial->parent), 0, &ucByte, 1);
	uartTransmit((unsigned char *)&ucByte, 1, 100);
    return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
//    serial->parent.read(&(serial->parent), 0, pucByte, 1);
//	uartread((unsigned char *)pucByte, 1, 100);
	*pucByte = uartRevChar;
    return TRUE;
}

/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR(void)
{
    pxMBMasterFrameCBTransmitterEmpty();
}

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR(void)
{
    pxMBMasterFrameCBByteReceived();
}

/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */
static void serial_soft_trans_irq(void const * parameter) {
    EventBits_t recved_event;
    while (1)
    {
        /* waiting for serial transmit start */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START, RT_EVENT_FLAG_OR,
//                RT_WAITING_FOREVER, &recved_event);
		
		recved_event = xEventGroupWaitBits(event_serial, EVENT_SERIAL_TRANS_START,
            pdFALSE, pdTRUE, portMAX_DELAY);
//		printf("\r\nSerial receive EVENT_SERIAL_TRANS_START");
        /* execute modbus callback */
        prvvUARTTxReadyISR();
    }
}

/**
 * This function is serial receive callback function
 *
 * @param dev the device of serial
 * @param size the data size that receive
 *
 * @return return RT_EOK
 */
unsigned int rec_count[20];
unsigned int count = 0;
static int serial_rx_ind(void) {
	//printf("\r\nserial_rx_ind");
	rec_count[count++] = uartRevChar;
	uart_receive_update();
    prvvUARTRxISR();
	
    return 0;
}

#endif
