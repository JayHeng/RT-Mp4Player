/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitLpuartPins();
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLpuartPins(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_25_LPUART1_RXD, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0x0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0x0u);
}

void BOARD_InitMipiPanelPins(void)
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_30_GPIO9_IO29, 0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_02_GPIO9_IO01, 0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_15_GPIO11_IO16, 0U);
}
