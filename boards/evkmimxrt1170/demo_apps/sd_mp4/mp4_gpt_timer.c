/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpt.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define GPT_IRQ_ID GPT2_IRQn
#define APP_GPT GPT2
#define APP_GPT_CLK_FREQ (CLOCK_GetFreq(kCLOCK_OscRc48MDiv2))
#define APP_GPT_IRQHandler GPT2_IRQHandler

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t s_gptFreq;
const uint32_t s_gptCompareValue = 0xffffffff;
volatile uint32_t s_highCounter;
volatile uint64_t s_gptLastTicks = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

//! @brief Read back running tick count
uint64_t gpt_get_ticks(void)
{
    uint64_t retVal;

    //! The rollover counter keeps track of increments higher than the 24 bit SysTick counter
    //! to combine them shift rollover up 24 bits and add the current ticks
    uint32_t high;
    uint32_t low;

    // Check for an overflow condition between the two reads above
    do
    {
        high = s_highCounter;
        low = GPT_GetCurrentTimerCount(APP_GPT);
    } while (high != s_highCounter);

    retVal = ((uint64_t)high * s_gptCompareValue) + low;

    return retVal;
}

uint64_t gpt_convert_to_ns(uint64_t ticks)
{
    return ((ticks * 1000) / (s_gptFreq / 1000000));
}

void gpt_delay(uint32_t us)
{
    uint64_t currentTicks = gpt_get_ticks();

    uint64_t ticksNeeded = ((uint64_t)us * s_gptFreq / 1000000) + currentTicks;
    while (gpt_get_ticks() < ticksNeeded);
}

void time_measure_start(void)
{
    s_gptLastTicks = gpt_get_ticks();
}

uint64_t time_measure_done(void)
{
    return gpt_convert_to_ns(gpt_get_ticks() - s_gptLastTicks);
}

void config_gpt(void)
{
    gpt_config_t gptConfig;
    
    /*Clock setting for GPT*/
    GPT_GetDefaultConfig(&gptConfig);

    /* Initialize GPT module */
    GPT_Init(APP_GPT, &gptConfig);

    /* Divide GPT clock source frequency by 1 inside GPT module */
    GPT_SetClockDivider(APP_GPT, 3);

    /* Get GPT clock frequency */
    s_gptFreq = APP_GPT_CLK_FREQ;

    /* GPT frequency is divided by 1 inside module */
    s_gptFreq /= 3;

    /* Set both GPT modules to 1 second duration */
    GPT_SetOutputCompareValue(APP_GPT, kGPT_OutputCompare_Channel1, s_gptCompareValue);

    /* Enable GPT Output Compare1 interrupt */
    GPT_EnableInterrupts(APP_GPT, kGPT_OutputCompare1InterruptEnable);

    /* Enable at the Interrupt */
    EnableIRQ(GPT_IRQ_ID);

    /* Start Timer */
    PRINTF("\r\nStarting GPT timer ...");
    s_highCounter = 0;
    GPT_StartTimer(APP_GPT);
}

void APP_GPT_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    GPT_ClearStatusFlags(APP_GPT, kGPT_OutputCompare1Flag);

    s_highCounter++;
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F, Cortex-M7, Cortex-M7F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
    __DSB();
#endif
}

