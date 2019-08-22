/*
 * Copyright 2019 NXP
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
    BOARD_InitPins();
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_25_LPUART1_RXD, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0x0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0x0u);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_03_GPIO9_IO02, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_15_GPIO9_IO14, 0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_34_USDHC1_VSELECT, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_00_USDHC1_CMD, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_01_USDHC1_CLK, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_USDHC1_DATA0, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_USDHC1_DATA1, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_04_USDHC1_DATA2, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_05_USDHC1_DATA3, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_03_GPIO9_IO02, 0x10B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_15_GPIO9_IO14, 0x017089u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_34_USDHC1_VSELECT, 0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_00_USDHC1_CMD, 4U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_01_USDHC1_CLK, 0xCU);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_02_USDHC1_DATA0, 4U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_USDHC1_DATA1, 4U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_04_USDHC1_DATA2, 4U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_05_USDHC1_DATA3, 4U);
}
