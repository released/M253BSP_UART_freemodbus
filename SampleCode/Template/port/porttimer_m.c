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
#include "NuMicro.h"
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#include "mbconfig.h"

extern uint32_t get_tick(void);

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Variables ----------------------------------------*/
//static USHORT usT35TimeOut50us;
//static struct rt_timer timer;
// static void prvvTIMERExpiredISR(void);
//static void timer_timeout_ind(void* parameter);

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR(void);

#define MODBUS_TIMER								(TIMER0)
#define MODBUS_TIMER_RST							(TMR0_RST)
#define MODBUS_TIMER_IRQn							(TMR0_IRQn)
#define MODBUS_TIMER_IRQHandler						(TMR0_IRQHandler)
/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit(USHORT usTimeOut50us)
{
	#if 1
    /* Enable Timer0 clock source */
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Select Timer0 clock source as external 12M */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Reset IP TMR0 */
    SYS_ResetModule(MODBUS_TIMER_RST);

    MODBUS_TIMER ->CTL = TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(MODBUS_TIMER, 0);
    TIMER_SET_CMP_VALUE(MODBUS_TIMER , usTimeOut50us*12*50);	

    /* Enable Timer0 interrupt */
    TIMER_EnableInt(MODBUS_TIMER);
    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(MODBUS_TIMER_IRQn);
    #else
    /* backup T35 ticks */
    usT35TimeOut50us = usTimeOut50us;

    rt_timer_init(&timer, "master timer",
                   timer_timeout_ind, /* bind timeout callback function */
                   RT_NULL,
                   (50 * usT35TimeOut50us) / (1000 * 1000 / RT_TICK_PER_SECOND) + 1,
                   RT_TIMER_FLAG_ONE_SHOT); /* one shot */
    
    #endif
    return TRUE;
}

void vMBMasterPortTimersT35Enable()
{
	#if 1
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_T35);

    TIMER_ResetCounter(MODBUS_TIMER);
	TIMER_Start(MODBUS_TIMER);
	#else
    rt_tick_t timer_tick = (50 * usT35TimeOut50us)
            / (1000 * 1000 / RT_TICK_PER_SECOND) + 1;

    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_T35);

    rt_timer_control(&timer, RT_TIMER_CTRL_SET_TIME, &timer_tick);

    rt_timer_start(&timer);	
	#endif
}

void vMBMasterPortTimersConvertDelayEnable()
{
	#if 1
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);

	TIMER_Stop(MODBUS_TIMER);
    TIMER_ResetCounter(MODBUS_TIMER);
	#else
    rt_tick_t timer_tick = MB_MASTER_DELAY_MS_CONVERT * RT_TICK_PER_SECOND / 1000;

    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);

    rt_timer_control(&timer, RT_TIMER_CTRL_SET_TIME, &timer_tick);

    rt_timer_start(&timer);	
	#endif
}

void vMBMasterPortTimersRespondTimeoutEnable()
{
	#if 1

    TIMER_ResetCounter(MODBUS_TIMER);    
    TIMER_Start(MODBUS_TIMER);
	#else
    rt_tick_t timer_tick = MB_MASTER_TIMEOUT_MS_RESPOND * RT_TICK_PER_SECOND / 1000;

    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);

    rt_timer_control(&timer, RT_TIMER_CTRL_SET_TIME, &timer_tick);

    rt_timer_start(&timer);
	#endif
}

void vMBMasterPortTimersDisable()
{
	#if 1
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);

	TIMER_Stop(MODBUS_TIMER);
    TIMER_ResetCounter(MODBUS_TIMER);
	#else
    rt_timer_stop(&timer);	
	#endif
}


void MODBUS_TIMER_IRQHandler(void)
{
 
   	TIMER_ClearIntFlag(MODBUS_TIMER);
	(void)pxMBMasterPortCBTimerExpired();

}

void prvvTIMERExpiredISR(void)
{
    (void) pxMBMasterPortCBTimerExpired();
}

static void timer_timeout_ind(void* parameter)
{
    prvvTIMERExpiredISR();
}

#endif
