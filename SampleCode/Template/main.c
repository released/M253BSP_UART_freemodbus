/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#include "misc_config.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mb_m.h"
#include "mbconfig.h"
#include "mbrtu.h"

#if defined (ENABLE_MODBUS_MASTER)
#include "mb_master.h"
#endif

#if defined (ENABLE_MODBUS_SLAVE)
#include "mb_slave.h"
#endif

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_500MS                   	(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;


// put under KEIL
// #define ENABLE_MODBUS_MASTER
// #define ENABLE_MODBUS_SLAVE

#if defined (ENABLE_MODBUS_SLAVE)
extern USHORT usSRegInBuf[S_REG_INPUT_NREGS];
#endif

#if defined (ENABLE_MODBUS_MASTER)
USHORT  usModbusUserData[MB_PDU_SIZE_MAX];
UCHAR   ucModbusUserData[MB_PDU_SIZE_MAX];
UCHAR   ucModbusDataFlag = 0;
#endif

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }


#if defined (ENABLE_MODBUS_MASTER)
eMBMasterReqErrCode 
eMBEmulateMasterBehavior(void)
{
    UCHAR   *ucMBFrame;
    UCHAR    ucRcvAddress;
    USHORT   usLength;
    eMBErrorCode    eStatus = MB_ENOERR;

    eMBMasterReqErrCode errorCode = MB_MRE_NO_ERR;    
    //Test Modbus Master
    // usSRegHoldBuf[S_HD_CPU_USAGE_MAJOR] = CpuUsageMajor;
    // usSRegHoldBuf[S_HD_CPU_USAGE_MINOR] = CpuUsageMinor;
    // usModbusUserData[0] = (USHORT)(rt_tick_get()/10);
    // usModbusUserData[1] = (USHORT)(rt_tick_get()%10);
    // ucModbusUserData[0] = 0x1F;
    // errorCode = eMBMasterReqReadDiscreteInputs(0x0A,1000,100,0);
    // errorCode = eMBMasterReqWriteMultipleCoils(1,3,5,ucModbusUserData,RT_WAITING_FOREVER);
    // errorCode = eMBMasterReqWriteCoil(1,8,0xFF00,RT_WAITING_FOREVER);
    // errorCode = eMBMasterReqReadCoils(0x0A,1000,100,0);
    errorCode = eMBMasterReqReadInputRegister(0x0A,M_REG_INPUT_START,M_REG_INPUT_NREGS,0);
    // errorCode = eMBMasterReqWriteHoldingRegister(0x0A,M_REG_HOLDING_START,M_REG_HOLDING_NREGS,0);
    // errorCode = eMBMasterReqWriteMultipleHoldingRegister(0x0A,M_REG_HOLDING_START,M_REG_HOLDING_NREGS,usModbusUserData,0);
    // errorCode = eMBMasterReqReadHoldingRegister(0x0A,M_REG_HOLDING_START,M_REG_HOLDING_NREGS,0);
    // errorCode = eMBMasterReqReadWriteMultipleHoldingRegister(0x0A,M_REG_HOLDING_START,M_REG_HOLDING_NREGS,usModbusUserData,M_REG_HOLDING_START,M_REG_HOLDING_NREGS,0);    

    if (errorCode != MB_MRE_NO_ERR) 
    {
        printf("mb : master error(0x%2X)\r\n" , errorCode);
    }
    else
    {
        printf("MB_EX_NONE\r\n");
        if (ucModbusDataFlag)
        {            
            ucModbusDataFlag = 0;
            eStatus = eMBMasterRTUReceive( &ucRcvAddress, &ucMBFrame, &usLength );
            printf("eStatus = 0x%2X,addr:0x%2X,len:0x%2X\r\n" , eStatus,ucRcvAddress,usLength);
            dump_buffer_hex(ucMBFrame,usLength);           
        }
    }

    return errorCode;    
}
#endif

#if defined (ENABLE_MODBUS_SLAVE)
eMBErrorCode
eMBEmulateSlaveData(void)
{
    eMBErrorCode eStatus = MB_ENOERR;
    static unsigned short cnt = 0;
    unsigned short i = 0;

    for(i = 0; i < S_REG_INPUT_NREGS; i++)
    {
        // usSRegInBuf[i] = i + 7000 + cnt;
        usSRegInBuf[i] = i + cnt;
        // usSRegHoldBuf[i] = i + cnt;
    }

    cnt++;

    //set indicator
    usSRegInBuf[0] = 0xA5A5;
    usSRegInBuf[99] = 0x5A5A;
    // usSRegHoldBuf[0] = 0xA5A5;
    // usSRegHoldBuf[99] = 0x5A5A;

    return eStatus;
}
#endif

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 500) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_500MS = 1;
		}	

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB0 ^= 1;      

        #if defined (ENABLE_MODBUS_SLAVE)
        eMBEmulateSlaveData();
        #endif
    }

    if (FLAG_PROJ_TIMER_PERIOD_500MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_500MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        #if defined (ENABLE_MODBUS_MASTER)
        (void)eMBMasterPoll();        
        eMBEmulateMasterBehavior();
        #endif

    }


    #if defined (ENABLE_MODBUS_MASTER)
    // (void)eMBMasterPoll();
    #endif

    #if defined (ENABLE_MODBUS_SLAVE)
    (void)eMBPoll();
    #endif



    /* Here we simply count the number of poll cycles. */
    // usRegInputBuf[0]++;
    

}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART4);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART4_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART4, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART4) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART4->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART4, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART4_Init(void)
{
    SYS_ResetModule(UART4_RST);

    /* Configure UART4 and set UART4 baud rate */
    UART_Open(UART4, 115200);
    UART_EnableInt(UART4, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART4_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | (SYS_GPB_MFPL_PB0MFP_GPIO);

    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(GPB_MODULE);
    
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Debug UART clock setting */
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();

    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M251 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART4 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    eMBErrorCode eStatus;

    SYS_Init();

	GPIO_Init();
	UART4_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    #if defined (ENABLE_MODBUS_MASTER)
    eStatus = eMBMasterInit(MB_RTU, 1, 115200,  MB_PAR_NONE);
    printf("M:eMBInit status : 0x%2X\r\n",eStatus);
    eStatus = eMBMasterEnable();
    printf("M:eMBEnable status : 0x%2X\r\n",eStatus);

    #endif

    #if defined (ENABLE_MODBUS_SLAVE)
    eStatus = eMBInit(MB_RTU, 0x0A, 0, 115200, MB_PAR_NONE);
    printf("S:eMBInit status : 0x%2X\r\n",eStatus);

    /* Enable the Modbus Protocol Stack. */
    eStatus = eMBEnable();
    printf("S:eMBEnable status : 0x%2X\r\n",eStatus);
    #endif

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
