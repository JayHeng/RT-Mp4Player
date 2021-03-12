/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "clock_config.h"
#include "fsl_iomuxc.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/

#ifndef SKIP_DCDC_ADJUSTMENT
#if __CORTEX_M == 4
#define SKIP_DCDC_ADJUSTMENT 1
#endif
#endif

#define DCDC_TARGET_VOLTAGE_1V 1150
#ifndef DCDC_TARGET_VOLTAGE
#define DCDC_TARGET_VOLTAGE DCDC_TARGET_VOLTAGE_1V
#endif

/*
 * TODO:
 *   replace the following two dcdc function with SDK DCDC driver when it's
 * ready
 */
/*******************************************************************************
 * Code for getting current DCDC voltage setting
 ******************************************************************************/
uint32_t dcdc_get_target_voltage()
{
    uint32_t temp = DCDC->DCDC_CTRL1;
    temp          = (temp & DCDC_DCDC_CTRL1_DCDC_VDD1P0CTRL_TRG_MASK) >> DCDC_DCDC_CTRL1_DCDC_VDD1P0CTRL_TRG_SHIFT;
    return (temp * 25 + 600);
}

/*******************************************************************************
 * Code for setting DCDC to target voltage
 ******************************************************************************/
void dcdc_trim_target_1p0(uint32_t target_voltage)
{
    uint8_t trim_value;
    uint32_t temp;

    trim_value = (target_voltage - 600) / 25;
    temp       = DCDC->DCDC_CTRL1;
    if ((temp & DCDC_DCDC_CTRL1_DCDC_VDD1P0CTRL_TRG(trim_value)) == DCDC_DCDC_CTRL1_DCDC_VDD1P0CTRL_TRG(trim_value))
    {
        return;
    }

    temp &= ~DCDC_DCDC_CTRL1_DCDC_VDD1P0CTRL_TRG_MASK;
    temp |= DCDC_DCDC_CTRL1_DCDC_VDD1P0CTRL_TRG(trim_value);
    DCDC->DCDC_CTRL1 = temp;
}

/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    clock_root_config_t rootCfg = {0};

#if !defined(SKIP_DCDC_ADJUSTMENT) || (!SKIP_DCDC_ADJUSTMENT)
    dcdc_trim_target_1p0(DCDC_TARGET_VOLTAGE);
#endif

#if defined(BYPASS_LDO_LPSR) && BYPASS_LDO_LPSR
    CLOCK_ANATOP_LdoLpsrAnaBypassOn();
    CLOCK_ANATOP_LdoLpsrDigBypassOn();
#endif

#if __CORTEX_M == 7
    // From RM: ARM_PLL_CTRL register defn
    // arm_pll_out = (24MHz * loopDivider) / 2.0 / postDivier
    // 104 <= loopDivider <= 208
    const clock_arm_pll_config_t armPllConfig = {
        /* ARM PLL 996 MHz. */
        //.postDivider = kCLOCK_PllPostDiv2,
        //.loopDivider = 166,
        /* ARM PLL 600 MHz. */
        .postDivider = kCLOCK_PllPostDiv4,
        .loopDivider = 200,
    };
#endif

    /* SYS PLL2 528MHz. */
    const clock_sys_pll_config_t sysPllConfig = {
        .loopDivider = 1,
        /* Using 24Mhz OSC */
        .mfn = 0,
        .mfi = 22,
    };

    const clock_sys_pll3_config_t sysPll3Config = {
        .divSelect = 3,
    };

    /* PLL LDO shall be enabled first before enable PLLs */
    CLOCK_EnableOsc24M();

#if __CORTEX_M == 7
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_M7, &rootCfg);
    CLOCK_SetRootClock(kCLOCK_Root_M7_Systick, &rootCfg);

    /* ARMPll: 996M */
    CLOCK_InitArmPll(&armPllConfig);
    /* Configure M7 */
    rootCfg.mux = 4;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_M7, &rootCfg);

    /* Configure M7 Systick running at 10K */
    rootCfg.mux = 0;
    rootCfg.div = 239;
    CLOCK_SetRootClock(kCLOCK_Root_M7_Systick, &rootCfg);
#endif
    CLOCK_InitSysPll2(&sysPllConfig);
    CLOCK_InitSysPll3(&sysPll3Config);

#if __CORTEX_M == 4
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_M4, &rootCfg);
    CLOCK_SetRootClock(kCLOCK_Root_Bus_Lpsr, &rootCfg);

    CLOCK_InitPfd(kCLOCK_Pll_SysPll3, kCLOCK_Pfd3, 18);
    /* Configure M4 using SysPll3Pfd3 divided by 1 */
    rootCfg.mux = 4;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_M4, &rootCfg);

    /* SysPll3 divide by 4 */
    rootCfg.mux = 5;
    rootCfg.div = 3;
    CLOCK_SetRootClock(kCLOCK_Root_Bus_Lpsr, &rootCfg);
#endif

#if DEBUG_CONSOLE_UART_INDEX == 1
    /* Configure Lpuart1 using Osc48MDiv2 */
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_Lpuart1, &rootCfg);
#else
    /* Configure Lpuart2 using Osc48MDiv2 */
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_Lpuart2, &rootCfg);
#endif

    CLOCK_EnableOscRc400M();
    /* Configure Semc using OscRc400M divided by 2 */
    //rootCfg.mux = 2;
    //rootCfg.div = 1;
    //CLOCK_InitPfd(kCLOCK_Pll_SysPll2, kCLOCK_Pfd1, 24);
    CLOCK_InitPfd(kCLOCK_Pll_SysPll2, kCLOCK_Pfd1, 29);
    // SysPll2Pfd1 = SysPll2 * 18 / PFD1_FRAC (13 - 35)
    /* Configure Semc using SysPll2Pfd1 divided by 2 */
    rootCfg.mux = 6;
    rootCfg.div = 1;
    CLOCK_SetRootClock(kCLOCK_Root_Semc, &rootCfg);

    /* Configure Bus using SysPll3 divided by 2 */
    rootCfg.mux = 4;
    rootCfg.div = 1;
    CLOCK_SetRootClock(kCLOCK_Root_Bus, &rootCfg);

    /* Configure Lpi2c1 using Osc48MDiv2 */
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_Lpi2c1, &rootCfg);

    /* Configure Lpi2c1 using Osc48MDiv2 */
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_Lpi2c5, &rootCfg);

    /* Configure gpt timer using Osc48MDiv2 */
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_Gpt2, &rootCfg);

    /* Configure lpspi using Osc48MDiv2 */
    rootCfg.mux = 0;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_Lpspi1, &rootCfg);

#if __CORTEX_M == 7
    SystemCoreClock = CLOCK_GetRootClockFreq(kCLOCK_Root_M7);
#else
    SystemCoreClock = CLOCK_GetRootClockFreq(kCLOCK_Root_M4);
#endif
}
