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

#include "NuMicro.h"
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#include "mbconfig.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/
//ALIGN(RT_ALIGN_SIZE)
/* software simulation serial transmit IRQ handler thread stack */
//static rt_uint8_t serial_soft_trans_irq_stack[512];
/* software simulation serial transmit IRQ handler thread */
//static struct rt_thread thread_serial_soft_trans_irq;
/* serial event */
//static struct rt_event event_serial;
/* modbus master serial device */
//static struct rt_serial_device *serial;

/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
//#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);
//static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size);
//static void serial_soft_trans_irq(void* parameter);

#define MODBUS_UART_PORT								(UART0)
#define MODBUS_UART_PORT_RST							(UART0_RST)
#define MODBUS_UART_PORT_IRQn							(UART0_IRQn)
#define MODBUS_UART_PORT_IRQHandler						(UART0_IRQHandler)

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{
	#if 1
    // if(ucPORT != 0) return FALSE;

    /* Step 1. GPIO initial */
    /* Set PB multi-function pins for UART0 RXD, TXD */
    /* PB.0 --> UART0 RX, PB.1 --> UART0 TX */
    // SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Step 2. Enable and Select UART clock source*/
    /* Enable UART0 clock */
    // CLK_EnableModuleClock(UART0_MODULE);
    /* UART0 clock source */
    // CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_HIRC;

    /* Step 3. Select Operation mode */
    SYS_ResetModule(MODBUS_UART_PORT_RST);
    // UART0->FCR |= 1 << 2;       //Tx FIFO Reset
    // UART0->FCR |= 1 << 1;       //Rx FIFO Reset

    MODBUS_UART_PORT->FIFO |= UART_FIFO_RXRST_Msk;
    while(MODBUS_UART_PORT->FIFO & UART_FIFO_RXRST_Msk);
    MODBUS_UART_PORT->FIFO |= UART_FIFO_TXRST_Msk;
    while(MODBUS_UART_PORT->FIFO & UART_FIFO_TXRST_Msk);    


    /* Step 4. Set Baud-Rate to 38400*/
    /* Configure UART0 and set UART0 Baud-rate: Baud-rate 38400, 8 bits length, None parity , 1 stop bit*/
    UART_Open(MODBUS_UART_PORT, ulBaudRate);

    switch(eParity)
    {
        case MB_PAR_NONE:
            MODBUS_UART_PORT->LINE &= ~UART_PARITY_EVEN;
            break;
        case MB_PAR_EVEN:
            MODBUS_UART_PORT->LINE &= ~UART_PARITY_EVEN;
            MODBUS_UART_PORT->LINE |= UART_PARITY_EVEN;
            break;
        case MB_PAR_ODD:
            MODBUS_UART_PORT->LINE &= ~UART_PARITY_EVEN;
            MODBUS_UART_PORT->LINE |= UART_PARITY_ODD;
            break;
    }

    MODBUS_UART_PORT->LINE &= ~UART_LINE_WLS_Msk;
    switch(ucDataBits)
    {
        case 5:
            MODBUS_UART_PORT->LINE |= UART_WORD_LEN_5;      //5 bits Data Length
            break;
        case 6:
            MODBUS_UART_PORT->LINE |= UART_WORD_LEN_6;      //6 bits Data Length
            break;
        case 7:
            MODBUS_UART_PORT->LINE |= UART_WORD_LEN_7;      //7 bits Data Length
            break;
        case 8:
            MODBUS_UART_PORT->LINE |= UART_WORD_LEN_8;      //8 bits Data Length
            break;
    }
	#else
    rt_device_t dev = RT_NULL;
    char uart_name[20];

    /**
     * set 485 mode receive and transmit control IO
     * @note MODBUS_MASTER_RT_CONTROL_PIN_INDEX need be defined by user
     */
#if defined(RT_MODBUS_MASTER_USE_CONTROL_PIN)
    rt_pin_mode(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_MODE_OUTPUT);
#endif
    /* set serial name */
    rt_snprintf(uart_name,sizeof(uart_name), "uart%d", ucPORT);

    dev = rt_device_find(uart_name);
    if(dev == RT_NULL)
    {
        /* can not find uart */
        return FALSE;
    }
    else
    {
        serial = (struct rt_serial_device*)dev;
    }

    /* set serial configure parameter */
    serial->config.baud_rate = ulBaudRate;
    serial->config.stop_bits = STOP_BITS_1;
    switch(eParity){
    case MB_PAR_NONE: {
        serial->config.data_bits = DATA_BITS_8;
        serial->config.parity = PARITY_NONE;
        break;
    }
    case MB_PAR_ODD: {
        serial->config.data_bits = DATA_BITS_9;
        serial->config.parity = PARITY_ODD;
        break;
    }
    case MB_PAR_EVEN: {
        serial->config.data_bits = DATA_BITS_9;
        serial->config.parity = PARITY_EVEN;
        break;
    }
    }
    /* set serial configure */
    serial->ops->configure(serial, &(serial->config));

    /* open serial device */
    if (!rt_device_open(&serial->parent, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX)) {
        rt_device_set_rx_indicate(&serial->parent, serial_rx_ind);
    } else {
        return FALSE;
    }

    /* software initialize */
    rt_event_init(&event_serial, "master event", RT_IPC_FLAG_PRIO);
    rt_thread_init(&thread_serial_soft_trans_irq,
                   "master trans",
                   serial_soft_trans_irq,
                   RT_NULL,
                   serial_soft_trans_irq_stack,
                   sizeof(serial_soft_trans_irq_stack),
                   10, 5);
    rt_thread_startup(&thread_serial_soft_trans_irq);
	
	#endif
    return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
	#if 1
    __disable_irq();

    if(xRxEnable)
    {
        UART_ENABLE_INT(MODBUS_UART_PORT, UART_INTEN_RDAIEN_Msk|UART_INTEN_RXTOIEN_Msk);

        // RS485 IO low
    }
    else
    {
        UART_DISABLE_INT(MODBUS_UART_PORT, UART_INTEN_RDAIEN_Msk|UART_INTEN_RXTOIEN_Msk);

        // RS485 IO high 
    }
    if(xTxEnable)
    {
        UART_ENABLE_INT(MODBUS_UART_PORT, UART_INTEN_THREIEN_Msk);
    }
    else
    {
        UART_DISABLE_INT(MODBUS_UART_PORT, UART_INTEN_THREIEN_Msk);
    }

    NVIC_EnableIRQ(MODBUS_UART_PORT_IRQn);
    __enable_irq();
    #else
    rt_uint32_t recved_event;
    if (xRxEnable)
    {
        /* enable RX interrupt */
        serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        /* switch 485 to receive mode */
#if defined(RT_MODBUS_MASTER_USE_CONTROL_PIN)
        rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_LOW);
#endif
    }
    else
    {
        /* switch 485 to transmit mode */
#if defined(RT_MODBUS_MASTER_USE_CONTROL_PIN)
        rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_HIGH);
#endif
        /* disable RX interrupt */
        serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }
    if (xTxEnable)
    {
        /* start serial transmit */
        rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
    }
    else
    {
        /* stop serial transmit */
        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
                &recved_event);
    }
    
    #endif
}

void vMBMasterPortClose(void)
{
	#if 1
    /* Disable Interrupt */
    UART_DISABLE_INT(MODBUS_UART_PORT, UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk|UART_INTEN_RXTOIEN_Msk);
    NVIC_DisableIRQ(MODBUS_UART_PORT_IRQn);
    #else
    serial->parent.close(&(serial->parent));    
    #endif
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
	#if 1
    // MODBUS_UART_PORT->THR = (uint8_t) ucByte;
    // while(UART_GET_TX_EMPTY(MODBUS_UART_PORT) != 0x00); //check Tx Empty

    while(MODBUS_UART_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
    MODBUS_UART_PORT->DAT = ucByte;
    #else
    serial->parent.write(&(serial->parent), 0, &ucByte, 1);    
	#endif
    return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
	#if 1
    // while(UART_GET_RX_EMPTY(UART0) != 0x00); //check Rx Empty
    // *pucByte = UART0->RBR;


    while((MODBUS_UART_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0);
    *pucByte = MODBUS_UART_PORT->DAT;
    #else
    serial->parent.read(&(serial->parent), 0, pucByte, 1);
    #endif
    return TRUE;
}


void MODBUS_UART_PORT_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(MODBUS_UART_PORT, UART_INTSTS_RDAINT_Msk) || 
        UART_GET_INT_FLAG(MODBUS_UART_PORT, UART_INTSTS_RXTOINT_Msk))
    {

        /* Get all the input characters */
        while(UART_IS_RX_READY(MODBUS_UART_PORT))
        {
            prvvUARTRxISR();
        }

    }
    else if(UART_GET_INT_FLAG(MODBUS_UART_PORT, UART_INTSTS_THREINT_Msk))
    {

        prvvUARTTxReadyISR();

    }
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

#if 0
/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */
static void serial_soft_trans_irq(void* parameter) {
    rt_uint32_t recved_event;
    while (1)
    {
        /* waiting for serial transmit start */
        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START, RT_EVENT_FLAG_OR,
                RT_WAITING_FOREVER, &recved_event);
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
static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size) {
    prvvUARTRxISR();
    return RT_EOK;
}

#endif
#endif
