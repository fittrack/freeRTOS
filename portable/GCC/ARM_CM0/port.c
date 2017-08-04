/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
    *                                                                       *
    *    FreeRTOS provides completely free yet professionally developed,    *
    *    robust, strictly quality controlled, supported, and cross          *
    *    platform software that is more than just the market leader, it     *
    *    is the industry's de facto standard.                               *
    *                                                                       *
    *    Help yourself get started quickly while simultaneously helping     *
    *    to support the FreeRTOS project by purchasing a FreeRTOS           *
    *    tutorial book, reference manual, or both:                          *
    *    http://www.FreeRTOS.org/Documentation                              *
    *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
* Implementation of functions defined in portable.h for the ARM CM0 port.
*----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_tim.h"
#include "cmsis_os.h"

#ifndef configSYSTICK_CLOCK_HZ
    #define configTIMER_CLOCK_HZ (32768)
#endif

/* Constants required to manipulate the NVIC. */
#define portNVIC_INT_CTRL			( ( volatile uint32_t *) 0xe000ed04 )
#define portNVIC_SYSPRI2			( ( volatile uint32_t *) 0xe000ed20 )
#define portNVIC_SYSTICK_COUNT_FLAG	( 1UL << 16UL )
#define portNVIC_PENDSVSET			0x10000000
#define portMIN_INTERRUPT_PRIORITY	( 255UL )
#define portNVIC_PENDSV_PRI			( portMIN_INTERRUPT_PRIORITY << 16UL )

TIM_HandleTypeDef        htim2;


/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR			( 0x01000000 )

/* The timer is a 16-bit counter. */
#define portMAX_16_BIT_NUMBER				( 0xffffUL )

/* A fiddle factor to estimate the number of SysTick counts that would have
occurred while the SysTick counter is stopped during tickless idle
calculations. */
#define portMISSED_COUNTS_FACTOR			( 128UL )

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS	prvTaskExitError
#endif

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
* Setup the timer to generate the tick interrupts.  The implementation in this
* file is weak to allow application writers to change the timer used to
* generate the tick interrupt.
*/
void vPortSetupTimerInterrupt( void );

/*
* Exception handlers.
*/
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void );

/*
* Start first task is a separate function so it can be tested in isolation.
*/
static void vPortStartFirstTask( void ) __attribute__ (( naked ));

/*
* Used to catch tasks that attempt to return from their implementing function.
*/
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/*
* The number of timer increments that make up one tick period.
*/
#if configUSE_TICKLESS_IDLE == 1
    static unsigned long ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
* The maximum number of tick periods that can be suppressed is limited by the
* 16 bit resolution of the timer.
*/
#if configUSE_TICKLESS_IDLE == 1
    static unsigned long xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
* Compensate for the CPU cycles that pass while the SysTick is stopped (low
* power functionality only.
*/
#if configUSE_TICKLESS_IDLE == 1
    static unsigned long ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*-----------------------------------------------------------*/

/*
* See header file for description.
*/
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
    /* Simulate the stack frame as it would be created by a context switch
    interrupt. */
    pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
    *pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pxCode;	/* PC */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;	/* LR */
    pxTopOfStack -= 5;	/* R12, R3, R2 and R1. */
    *pxTopOfStack = ( StackType_t ) pvParameters;	/* R0 */
    pxTopOfStack -= 8; /* R11..R4. */

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
    /* A function that implements a task must not exit or attempt to return to
    its caller as there is nothing to return to.  If a task wants to exit it
    should instead call vTaskDelete( NULL ).

    Artificially force an assert() to be triggered if configASSERT() is
    defined, then stop here so application writers can catch the error. */
    configASSERT( uxCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
    /* This function is no longer used, but retained for backward
    compatibility. */
}
/*-----------------------------------------------------------*/

void vPortStartFirstTask( void )
{
    /* The MSP stack is not reset as, unlike on M3/4 parts, there is no vector
    table offset register that can be used to locate the initial stack value.
    Not all M0 parts have the application vector table at address 0. */
    __asm volatile(
        "	ldr	r2, pxCurrentTCBConst2	\n" /* Obtain location of pxCurrentTCB. */
        "	ldr r3, [r2]				\n"
        "	ldr r0, [r3]				\n" /* The first item in pxCurrentTCB is the task top of stack. */
        "	add r0, #32					\n" /* Discard everything up to r0. */
        "	msr psp, r0					\n" /* This is now the new top of stack to use in the task. */
        "	movs r0, #2					\n" /* Switch to the psp stack. */
        "	msr CONTROL, r0				\n"
        "	isb							\n"
        "	pop {r0-r5}					\n" /* Pop the registers that are saved automatically. */
        "	mov lr, r5					\n" /* lr is now in r5. */
        "	pop {r3}					\n" /* Return address is now in r3. */
        "	pop {r2}					\n" /* Pop and discard XPSR. */
        "	cpsie i						\n" /* The first task has its context and interrupts can be enabled. */
        "	bx r3						\n" /* Finally, jump to the user defined task code. */
        "								\n"
        "	.align 4					\n"
        "pxCurrentTCBConst2: .word pxCurrentTCB	  "
    );
}
/*-----------------------------------------------------------*/

/*
* See header file for description.
*/
BaseType_t xPortStartScheduler( void )
{
    /* Make PendSV, CallSV and SysTick the same priroity as the kernel. */
    *(portNVIC_SYSPRI2) |= portNVIC_PENDSV_PRI;

    /* Start the timer that generates the tick ISR.  Interrupts are disabled
    here already. */
    vPortSetupTimerInterrupt();

    /* Initialise the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;

    /* Start the first task. */
    vPortStartFirstTask();

    /* Should never get here as the tasks will now be executing!  Call the task
    exit error function to prevent compiler warnings about a static function
    not being called in the case that the application writer overrides this
    functionality by defining configTASK_RETURN_ADDRESS. */
    prvTaskExitError();

    /* Should not get here! */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is nothing to return to.
    Artificially force an assert. */
    configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortYield( void )
{
    /* Set a PendSV to request a context switch. */
    *( portNVIC_INT_CTRL ) = portNVIC_PENDSVSET;

    /* Barriers are normally not required but do ensure the code is completely
    within the specified behaviour for the architecture. */
    __asm volatile( "dsb" );
    __asm volatile( "isb" );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
    __asm volatile( "dsb" );
    __asm volatile( "isb" );
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;
    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/

uint32_t ulSetInterruptMaskFromISR( void )
{
    __asm volatile(
        " mrs r0, PRIMASK	\n"
        " cpsid i			\n"
        " bx lr				  "
    );

    /* To avoid compiler warnings.  This line will never be reached. */
    return 0;
}
/*-----------------------------------------------------------*/

void vClearInterruptMaskFromISR( uint32_t ulMask )
{
    __asm volatile(
        " msr PRIMASK, r0	\n"
        " bx lr				  "
    );

    /* Just to avoid compiler warning. */
    ( void ) ulMask;
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
    /* This is a naked function. */

    __asm volatile
    (
        "	mrs r0, psp							\n"
        "										\n"
        "	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
        "	ldr	r2, [r3]						\n"
        "										\n"
        "	sub r0, r0, #32						\n" /* Make space for the remaining low registers. */
        "	str r0, [r2]						\n" /* Save the new top of stack. */
        "	stmia r0!, {r4-r7}					\n" /* Store the low registers that are not saved automatically. */
        " 	mov r4, r8							\n" /* Store the high registers. */
        " 	mov r5, r9							\n"
        " 	mov r6, r10							\n"
        " 	mov r7, r11							\n"
        " 	stmia r0!, {r4-r7}              	\n"
        "										\n"
        "	push {r3, r14}						\n"
        "	cpsid i								\n"
        "	bl vTaskSwitchContext				\n"
        "	cpsie i								\n"
        "	pop {r2, r3}						\n" /* lr goes in r3. r2 now holds tcb pointer. */
        "										\n"
        "	ldr r1, [r2]						\n"
        "	ldr r0, [r1]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
        "	add r0, r0, #16						\n" /* Move to the high registers. */
        "	ldmia r0!, {r4-r7}					\n" /* Pop the high registers. */
        " 	mov r8, r4							\n"
        " 	mov r9, r5							\n"
        " 	mov r10, r6							\n"
        " 	mov r11, r7							\n"
        "										\n"
        "	msr psp, r0							\n" /* Remember the new top of stack for the task. */
        "										\n"
        "	sub r0, r0, #32						\n" /* Go back for the low registers that are not automatically restored. */
        " 	ldmia r0!, {r4-r7}              	\n" /* Pop low registers.  */
        "										\n"
        "	bx r3								\n"
        "										\n"
        "	.align 4							\n"
        "pxCurrentTCBConst: .word pxCurrentTCB	  "
    );
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    uint32_t ulPreviousMask;

    ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            /* Pend a context switch. */
            *(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );
}
/*-----------------------------------------------------------*/

#if configUSE_TICKLESS_IDLE == 1

__attribute__((weak)) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
    TickType_t xModifiableIdleTime;

    /* Make sure the SysTick reload value does not overflow the counter. */
    if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    /* Stop the Timer momentarily.  The time the SysTick is stopped for
    is accounted for as best it can be, but using the tickless mode will
    inevitably result in some tiny drift of the time maintained by the
    kernel with respect to calendar time. */
    __HAL_TIM_DISABLE(&htim2);

    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods. */

    ulReloadValue = __HAL_TIM_GET_COUNTER(&htim2) + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );
    if( ulReloadValue > ulStoppedTimerCompensation )
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __asm volatile( "cpsid i" );

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        /* Restart from whatever is left in the count register to complete
        this tick period. */

        /* Reset the reload register to the value required for normal tick
        periods. */
        __HAL_TIM_SET_AUTORELOAD(&htim2, ulTimerCountsForOneTick - 1UL);

        /* Restart Timer. */
        __HAL_TIM_ENABLE(&htim2);

        /* Re-enable interrupts - see comments above the cpsid instruction()
        above. */
        __asm volatile( "cpsie i" );
    }
    else
    {
        /* Set the new reload value. */
        __HAL_TIM_SET_AUTORELOAD(&htim2, ulReloadValue);
        __HAL_TIM_SET_COUNTER(&htim2, ulReloadValue);
        __HAL_TIM_ENABLE(&htim2);

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( &xModifiableIdleTime );
        if( xModifiableIdleTime > 0 )
        {
            __asm volatile( "dsb" );
            __asm volatile( "wfi" );
            __asm volatile( "isb" );
        }
        configPOST_SLEEP_PROCESSING( &xExpectedIdleTime );

        /* Stop Timer.  Again, the time the Timer is stopped for is
        accounted for as best it can be, but using the tickless mode will
        inevitably result in some tiny drift of the time maintained by the
        kernel with respect to calendar time. */

        __HAL_TIM_DISABLE(&htim2);
        uint8_t timer_interrupt = LL_TIM_IsActiveFlag_UPDATE(htim2.Instance);
        uint32_t current_counter = __HAL_TIM_GET_COUNTER(&htim2);

        /* Re-enable interrupts - see comments above the cpsid instruction()
        above. */
        __asm volatile( "cpsie i" );

        if( timer_interrupt )
        {
            uint32_t ulCalculatedLoadValue;

            /* The tick interrupt has already executed.  Reset the
            Timer counter with whatever remains of this tick
            period. */
            ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - current_counter );

            /* Don't allow a tiny value, or values that have somehow
            underflowed because the post sleep hook did something
            that took too long. */
            if( ( ulCalculatedLoadValue < ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
            {
                ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
            }

            __HAL_TIM_SET_COUNTER(&htim2, ulCalculatedLoadValue);

            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be
            processed as soon as this function exits, the tick value
            maintained by the tick is stepped forward by one less than the
            time spent waiting. */
            ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        }
        else
        {
            /* Something other than the tick interrupt ended the sleep.
            Work out how long the sleep lasted rounded to complete tick
            periods (not the ulReload value which accounted for part
            ticks). */
            uint32_t expected_ticks = xExpectedIdleTime * ulTimerCountsForOneTick;
            /* Make sure that the number of decrements is not negative.
             * Before we go to sleep we set the reload time to the sum of the current timer value and the requested ticks.
             * Now, if an external interrupt wakes us immediately after we go to sleep, the timer will have decremented only a few ticks.
             * That means that the current timer value is still greater than the requested ticks, which would make make the next result negative.
             * This negative value (truncated to a large positive one, because of uint32_t), would trigger an assert in vTaskStepTick */
            if(current_counter > expected_ticks)
            {
                ulCompletedSysTickDecrements = ulReloadValue - current_counter;
            }
            else
            {
                ulCompletedSysTickDecrements = expected_ticks - current_counter;
            }

            /* How many complete tick periods passed while the processor
            was waiting? */
            ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

            /* The counter value is set to whatever fraction of a single tick
            period remains. */
            __HAL_TIM_SET_COUNTER(&htim2, ( ( ulCompleteTickPeriods + 1 ) * ulTimerCountsForOneTick ) - ulCompletedSysTickDecrements);
        }

        portENTER_CRITICAL();
        {
            __HAL_TIM_SET_AUTORELOAD(&htim2, ulTimerCountsForOneTick - 1UL);
            __HAL_TIM_ENABLE(&htim2);
            vTaskStepTick( ulCompleteTickPeriods );
        }
        portEXIT_CRITICAL();
    }
}

#endif /* #if configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/

void TIM2_IRQHandler(void)
{
    if(LL_TIM_IsActiveFlag_UPDATE(htim2.Instance))
    {
        LL_TIM_ClearFlag_UPDATE(htim2.Instance);
        osSystickHandler();
    }
}

void vPortUpdateClockSettings(void)
{
    uint32_t uwTimclock = HAL_RCC_GetPCLK1Freq();
    uint32_t uwPrescalerValue = (uint32_t) ((uwTimclock / configTIMER_CLOCK_HZ) - 1);
    ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configTIMER_CLOCK_HZ );

    __HAL_TIM_SET_PRESCALER(&htim2, uwPrescalerValue);
    LL_TIM_GenerateEvent_UPDATE(htim2.Instance);
}

/*
* Setup the timer to generate the tick interrupts at the required
* frequency.
*/
__attribute__(( weak )) void vPortSetupTimerInterrupt( void )
{
    /* Calculate the constants required to configure the tick interrupt. */
    #if configUSE_TICKLESS_IDLE == 1
    {
    ulTimerCountsForOneTick = ( configTIMER_CLOCK_HZ / configTICK_RATE_HZ );
    xMaximumPossibleSuppressedTicks = portMAX_16_BIT_NUMBER / ulTimerCountsForOneTick;
    ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configTIMER_CLOCK_HZ );
    }
    #endif /* configUSE_TICKLESS_IDLE */

    __HAL_RCC_TIM2_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);

    /* Enable the TIM2 global Interrupt */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    htim2.Instance = TIM2;

    uint32_t uwTimclock = HAL_RCC_GetPCLK1Freq();
    uint32_t uwPrescalerValue = (uint32_t) ((uwTimclock / configTIMER_CLOCK_HZ) - 1);

    htim2.Init.Period = ulTimerCountsForOneTick - 1;
    htim2.Init.Prescaler = uwPrescalerValue;
    htim2.Init.ClockDivision = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    HAL_TIM_Base_Init(&htim2);
    __HAL_TIM_URS_ENABLE(&htim2);

    /* Start the TIM time Base generation in interrupt mode */
    HAL_TIM_Base_Start_IT(&htim2);
}
/*-----------------------------------------------------------*/

