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
 * File: $Id: porttimer.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "NuMicro.h"
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

extern uint32_t get_tick(void);
/* ----------------------- static functions ---------------------------------*/
// static struct rt_timer timer;
static void prvvTIMERExpiredISR(void);
static void timer_timeout_ind(void* parameter);

#define MODBUS_TIMER								(TIMER0)
#define MODBUS_TIMER_RST							(TMR0_RST)
#define MODBUS_TIMER_IRQn							(TMR0_IRQn)
#define MODBUS_TIMER_IRQHandler						(TMR0_IRQHandler)

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
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
    TIMER_SET_CMP_VALUE(MODBUS_TIMER , usTim1Timerout50us*12*50);	

    /* Enable Timer0 interrupt */
    TIMER_EnableInt(MODBUS_TIMER);
    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(MODBUS_TIMER_IRQn);
    #else
    rt_timer_init(&timer, "slave timer",
                   timer_timeout_ind, /* bind timeout callback function */
                   RT_NULL,
                   (50 * usTim1Timerout50us) / (1000 * 1000 / RT_TICK_PER_SECOND) + 1,
                   RT_TIMER_FLAG_ONE_SHOT); /* one shot */    
    #endif
    return TRUE;
}

void vMBPortTimersEnable()
{
	#if 1
    // /* Reset Timer0 counter */
    // MODBUS_TIMER->TCSR |= TIMER_TCSR_CRST_Msk;
    // /* Enable Timer0 */
    // MODBUS_TIMER->TCSR |= TIMER_TCSR_CEN_Msk;

    TIMER_ResetCounter(MODBUS_TIMER);
    TIMER_Start(MODBUS_TIMER);
    #else
    rt_timer_start(&timer);    
    #endif
}

void vMBPortTimersDisable()
{
	#if 1
    // /* Disable Timer0 */
    // MODBUS_TIMER->TCSR &= ~TIMER_TCSR_CEN_Msk;
    //  /* Reset Timer0 counter */
    // MODBUS_TIMER->TCSR |= TIMER_TCSR_CRST_Msk;

    TIMER_Stop(MODBUS_TIMER);
    TIMER_ResetCounter(MODBUS_TIMER);
    #else
    rt_timer_stop(&timer);    
    #endif
}

void vMBPortTimersDelay(USHORT usTimeOutMS)
{
    uint32_t tickstart = get_tick();
    uint32_t wait = usTimeOutMS;

    while ((get_tick() - tickstart) < wait)
    {
    }
}

void MODBUS_TIMER_IRQHandler(void)
{
    /* Clear Timer0 interrupt flag */
    // MODBUS_TIMER->TISR |= TIMER_TISR_TIF_Msk;
    TIMER_ClearIntFlag(MODBUS_TIMER);

    (void)pxMBPortCBTimerExpired();
}

void prvvTIMERExpiredISR(void)
{
    (void) pxMBPortCBTimerExpired();
}

static void timer_timeout_ind(void* parameter)
{
    prvvTIMERExpiredISR();
}
