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
 * File: $Id: porttimer_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Variables ----------------------------------------*/
static USHORT usT35TimeOut50us;
//static struct rt_timer timer;
static TimerHandle_t timer;
static void prvvTIMERExpiredISR(void);
static void timer_timeout_ind(void* parameter);

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR(void);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit(USHORT usTimeOut50us)
{
    /* backup T35 ticks */
    usT35TimeOut50us = usTimeOut50us;
	printf("\r\nxMBMasterPortTimersInit");
//    rt_timer_init(&timer, "master timer",
//                   timer_timeout_ind, /* bind timeout callback function */
//                   RT_NULL,
//                   (50 * usT35TimeOut50us) / (1000 * 1000 / RT_TICK_PER_SECOND),
//                   RT_TIMER_FLAG_ONE_SHOT); /* one shot */
	timer = xTimerCreate("master timer", pdMS_TO_TICKS((60 * usT35TimeOut50us)/1000), pdFALSE, (void *)0, timer_timeout_ind);
	if( timer == NULL ) {
		printf("\r\n xTimerCreate Fail.");
	}
//	xTimerStart(timer, 0);
    return TRUE;
}

void vMBMasterPortTimersT35Enable()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    rt_tick_t timer_tick = (50 * usT35TimeOut50us)
//            / (1000 * 1000 / RT_TICK_PER_SECOND);
	TickType_t timer_tick = pdMS_TO_TICKS((60 * usT35TimeOut50us)/1000);
//	printf("\r\nvMBMasterPortTimersT35Enable: timer_tick: %d",timer_tick);
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_T35);
	xTimerChangePeriodFromISR(timer, timer_tick, &xHigherPriorityTaskWoken);
//    xTimerReset(timer, 0);
//	xTimerStart(timer, 0);
	xTimerStartFromISR(timer, &xHigherPriorityTaskWoken);
}

void vMBMasterPortTimersConvertDelayEnable()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TickType_t timer_tick = MB_MASTER_DELAY_MS_CONVERT * configTICK_RATE_HZ / 1000;

	printf("\r\nvMBMasterPortTimersConvertDelayEnable: timer_tick: %d",timer_tick);
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);

    xTimerChangePeriodFromISR(timer, timer_tick, &xHigherPriorityTaskWoken);

//    xTimerReset(timer, 0);
//	xTimerStart(timer, 0);
	xTimerStartFromISR(timer, &xHigherPriorityTaskWoken);
}

void vMBMasterPortTimersRespondTimeoutEnable()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TickType_t timer_tick = MB_MASTER_TIMEOUT_MS_RESPOND * configTICK_RATE_HZ / 1000;
	
//	printf("\r\nvvMBMasterPortTimersRespondTimeoutEnable: timer_tick: %d",timer_tick);
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);

     xTimerChangePeriodFromISR(timer, timer_tick, &xHigherPriorityTaskWoken);

//    xTimerReset(timer, 0);
//	xTimerStart(timer, 0);
	xTimerStartFromISR(timer, &xHigherPriorityTaskWoken);
}

void vMBMasterPortTimersDisable()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	 if( xTimerIsTimerActive( timer ) != pdFALSE )
//		xTimerStop(timer, 1);
	xTimerStopFromISR(timer, &xHigherPriorityTaskWoken);
}

void prvvTIMERExpiredISR(void)
{
    (void) pxMBMasterPortCBTimerExpired();
}

static void timer_timeout_ind( TimerHandle_t xTimer)
{
	prvvTIMERExpiredISR();
//	xTimerReset(xTimer, 0);
}

#endif
