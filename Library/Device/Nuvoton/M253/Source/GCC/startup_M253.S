/****************************************************************************//**
 * @file     startup_M253.S
 * @version  V1.00
 * @brief    CMSIS Cortex-M23 Core Device Startup File for M253
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

	.syntax	unified
	.arch	armv8-m.base

	.section .stack
	.align	3
#ifdef __STACK_SIZE
	.equ	Stack_Size, __STACK_SIZE
#else
	.equ	Stack_Size, 0x00000400
#endif
	.globl	__StackTop
	.globl	__StackLimit
__StackLimit:
	.space	Stack_Size
	.size	__StackLimit, . - __StackLimit
__StackTop:
	.size	__StackTop, . - __StackTop

	.section .heap
	.align	3
#ifdef __HEAP_SIZE
	.equ	Heap_Size, __HEAP_SIZE
#else
	.equ	Heap_Size, 0x00000100
#endif
	.globl	__HeapBase
	.globl	__HeapLimit
__HeapBase:
	.if	Heap_Size
	.space	Heap_Size
	.endif
	.size	__HeapBase, . - __HeapBase
__HeapLimit:
	.size	__HeapLimit, . - __HeapLimit

	.section .vectors
	.align	2
	.globl	__Vectors
__Vectors:
	.long	__StackTop              /* Top of Stack                  */
	.long	Reset_Handler           /* Reset Handler                 */
	.long	NMI_Handler             /* NMI Handler                   */
	.long	HardFault_Handler       /* Hard Fault Handler            */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	SVC_Handler             /* SVCall Handler                */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	PendSV_Handler          /* PendSV Handler                */
	.long	SysTick_Handler         /* SysTick Handler               */

	/* External interrupts */
	.long	BOD_IRQHandler          /*  0: BOD                        */
	.long	IRCTRIM_IRQHandler      /*  1: IRC                        */
	.long	PWRWU_IRQHandler        /*  2: PWRWU                      */
	.long	Default_Handler         /*  3: Reserved                   */
	.long	CLKFAIL_IRQHandler      /*  4: CKFAIL                     */
	.long	Default_Handler         /*  5: Reserved                   */
	.long	RTC_IRQHandler          /*  6: RTC                        */
	.long	Default_Handler         /*  7: Reserved                   */
	.long	WDT_IRQHandler          /*  8: WDT                        */
	.long	WWDT_IRQHandler         /*  9: WWDT                       */
	.long	EINT0_IRQHandler        /* 10: EINT0                      */
	.long	EINT1_IRQHandler        /* 11: EINT1                      */
	.long	EINT2_IRQHandler        /* 12: EINT2                      */
	.long	EINT3_IRQHandler        /* 13: EINT3                      */
	.long	EINT4_IRQHandler        /* 14: EINT4                      */
	.long	EINT5_IRQHandler        /* 15: EINT5                      */
	.long	GPA_IRQHandler          /* 16: GPA                        */
	.long	GPB_IRQHandler          /* 17: GPB                        */
	.long	GPC_IRQHandler          /* 18: GPC                        */
	.long	GPD_IRQHandler          /* 19: GPD                        */
	.long	GPE_IRQHandler          /* 20: GPE                        */
	.long	GPF_IRQHandler          /* 21: GPF                        */
	.long	Default_Handler         /* 22: Reserved                   */
	.long	SPI0_IRQHandler         /* 23: SPI0                       */
	.long	CANFD0_IRQ0_IRQHandler  /* 24: CANDF0_IRQ0                */
	.long	CANFD0_IRQ1_IRQHandler  /* 25: CANDF0_IRQ1                */
	.long	Default_Handler         /* 26: Reserved                   */
	.long	Default_Handler         /* 27: Reserved                   */
	.long	Default_Handler         /* 28: Reserved                   */
	.long	Default_Handler         /* 29: Reserved                   */
	.long	Default_Handler         /* 30: Reserved                   */
	.long	Default_Handler         /* 31: Reserved                   */
	.long	TMR0_IRQHandler         /* 32: TIMER0                     */
	.long	TMR1_IRQHandler         /* 33: TIMER1                     */
	.long	TMR2_IRQHandler         /* 34: TIMER2                     */
	.long	TMR3_IRQHandler         /* 35: TIMER3                     */
	.long	UART0_IRQHandler        /* 36: UART0                      */
	.long	UART1_IRQHandler        /* 37: UART1                      */
	.long	I2C0_IRQHandler         /* 38: I2C0                       */
	.long	I2C1_IRQHandler         /* 39: I2C1                       */
	.long	PDMA_IRQHandler         /* 40: PDMA                       */
	.long	Default_Handler         /* 41: Reserved                   */
	.long	EADC_INT0_IRQHandler    /* 42: EADC00                     */
	.long	EADC_INT1_IRQHandler    /* 43: EADC01                     */
	.long	Default_Handler         /* 44: Reserved                   */
	.long	BPWM0_IRQHandler        /* 45: BPWM0                      */
	.long	EADC_INT2_IRQHandler    /* 46: EADC02                     */
	.long	EADC_INT3_IRQHandler    /* 47: EADC03                     */
	.long	UART2_IRQHandler        /* 48: UART2                      */
	.long	UART3_IRQHandler        /* 49: UART3                      */
	.long	USCI0_IRQHandler        /* 50: UCSI0                      */
	.long	UART4_IRQHandler        /* 51: UART4                      */
	.long	Default_Handler         /* 52: Reserved                   */
	.long	USBD_IRQHandler         /* 53: USBD                       */
	.long	Default_Handler         /* 54: Reserved                   */
	.long	Default_Handler         /* 55: Reserved                   */
	.long	Default_Handler         /* 56: Reserved                   */
	.long	Default_Handler         /* 57: Reserved                   */
	.long	Default_Handler         /* 58: Reserved                   */
	.long	Default_Handler         /* 59: Reserved                   */
	.long	Default_Handler         /* 60: Reserved                   */
	.long	Default_Handler         /* 61: Reserved                   */
	.long	Default_Handler         /* 62: Reserved                   */
	.long	Default_Handler         /* 63: Reserved                   */

	.size	__Vectors, . - __Vectors

	.text
	.thumb
	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r4, =__copy_table_start__
	ldr	r5, =__copy_table_end__

.L_loop0:
	cmp	r4, r5
	bge	.L_loop0_done
	ldr	r1, [r4]
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #8]

.L_loop0_0:
	subs	r3, #4
	blt	.L_loop0_0_done
	ldr	r0, [r1,r3]
	str	r0, [r2,r3]
	b	.L_loop0_0

.L_loop0_0_done:
	adds	r4, #12
	b	.L_loop0

.L_loop0_done:
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__

	subs	r3, r2
	ble	.L_loop1_done

.L_loop1:
	subs	r3, #4
	ldr	r0, [r1,r3]
	str	r0, [r2,r3]
	bgt	.L_loop1

.L_loop1_done:

#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise define macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
	ldr	r3, =__zero_table_start__
	ldr	r4, =__zero_table_end__

.L_loop2:
	cmp	r3, r4
	bge	.L_loop2_done
	ldr	r1, [r3]
	ldr	r2, [r3, #4]
	movs	r0, 0

.L_loop2_0:
	subs	r2, #4
	str 	r0, [r1, r2]
	bgt	.L_loop2_0

	adds	r3, #8
	b	.L_loop2
.L_loop2_done:

#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__

    movs    r0, 0

    subs    r2, r1
    ble .L_loop3_done

.L_loop3:
    subs    r2, #4
    str r0, [r1, r2]
    bgt .L_loop3
.L_loop3_done:
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */




/*  Unlock Register */
	ldr	r0, =0x40000100
	movw r1, 0x00000059
	str	r1, [r0]
	movw r1, 0x00000016
	str	r1, [r0]
	movw r1, 0x00000088
	str	r1, [r0]


#ifndef __NO_SYSTEM_INIT
	bl	SystemInit
#endif

/* Init POR */
#if 1

	ldr	r0, =0x40000024
	movw r1, 0x00005AA5
	str	r1, [r0]

	ldr	r0, =0x400001EC
	str	r1, [r0]
#endif

/* Lock register */
	ldr	r0, =0x40000100
	movw r1, 0x00000000
	str	r1, [r0]

#ifndef __START
#define __START _start
#endif
	bl	__START

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.align	1
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm
	def_irq_handler	NMI_Handler
    def_irq_handler	HardFault_Handler
    def_irq_handler	SVC_Handler
    def_irq_handler	PendSV_Handler
    def_irq_handler	SysTick_Handler
    def_irq_handler	BOD_IRQHandler
    def_irq_handler	IRCTRIM_IRQHandler
    def_irq_handler	PWRWU_IRQHandler
  //def_irq_handler	Default_Handler
    def_irq_handler	CLKFAIL_IRQHandler
  //def_irq_handler	Default_Handler
    def_irq_handler	RTC_IRQHandler
  //def_irq_handler	Default_Handler
    def_irq_handler	WDT_IRQHandler
    def_irq_handler	WWDT_IRQHandler
    def_irq_handler	EINT0_IRQHandler
    def_irq_handler	EINT1_IRQHandler
    def_irq_handler	EINT2_IRQHandler
    def_irq_handler	EINT3_IRQHandler
    def_irq_handler	EINT4_IRQHandler
    def_irq_handler	EINT5_IRQHandler
    def_irq_handler	GPA_IRQHandler
    def_irq_handler	GPB_IRQHandler
    def_irq_handler	GPC_IRQHandler
    def_irq_handler	GPD_IRQHandler
    def_irq_handler	GPE_IRQHandler
    def_irq_handler	GPF_IRQHandler
  //def_irq_handler	Default_Handler
    def_irq_handler	SPI0_IRQHandler
    def_irq_handler	CANFD0_IRQ0_IRQHandler
    def_irq_handler	CANFD0_IRQ1_IRQHandler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
    def_irq_handler	TMR0_IRQHandler
    def_irq_handler	TMR1_IRQHandler
    def_irq_handler	TMR2_IRQHandler
    def_irq_handler	TMR3_IRQHandler
    def_irq_handler	UART0_IRQHandler
    def_irq_handler	UART1_IRQHandler
    def_irq_handler	I2C0_IRQHandler
    def_irq_handler	I2C1_IRQHandler
    def_irq_handler	PDMA_IRQHandler
  //def_irq_handler	Default_Handler
    def_irq_handler	EADC_INT0_IRQHandler
    def_irq_handler	EADC_INT1_IRQHandler
  //def_irq_handler	Default_Handler
    def_irq_handler	BPWM0_IRQHandler
    def_irq_handler	EADC_INT2_IRQHandler
    def_irq_handler	EADC_INT3_IRQHandler
    def_irq_handler	UART2_IRQHandler
    def_irq_handler	UART3_IRQHandler
    def_irq_handler	USCI0_IRQHandler
    def_irq_handler	UART4_IRQHandler
  //def_irq_handler	Default_Handler
    def_irq_handler	USBD_IRQHandler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
  //def_irq_handler	Default_Handler
    
    
    
    
    
    
    
    
    
    
    
    
    


    .end
