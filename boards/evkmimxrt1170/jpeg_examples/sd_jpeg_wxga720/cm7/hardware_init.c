/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*${header:start}*/
#include "pin_mux.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_common.h"
/*${header:end}*/

/*${function:start}*/
static void BOARD_USDHCClockConfiguration(void)
{
    clock_root_config_t rootCfg = {0};

    CLOCK_EnableOscRc400M();
    /* Configure USDHC1 using RC400M divided by 1 */
    rootCfg.mux = 2;
    rootCfg.div = 0;
    CLOCK_SetRootClock(kCLOCK_Root_Usdhc1, &rootCfg);
}

void BOARD_InitHardware(void)
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_USDHCClockConfiguration();
    BOARD_InitDebugConsole();
}
/*${function:end}*/
