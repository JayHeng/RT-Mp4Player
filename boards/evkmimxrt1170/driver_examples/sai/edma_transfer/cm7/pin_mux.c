/*
 * Copyright 2018 NXP
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
void BOARD_InitPins(void) {
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_24_LPUART1_TXD,0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_25_LPUART1_RXD,0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_24_LPUART1_TXD,0x10B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_25_LPUART1_RXD,0x10B0u);
    IOMUXC_SetPinMux(IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_17_SAI1_MCLK,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_21_SAI1_TX_DATA00,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_22_SAI1_TX_BCLK,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_23_SAI1_TX_SYNC,1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,0xD8B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,0xD8B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_17_SAI1_MCLK,0x6u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_21_SAI1_TX_DATA00,0x6u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_22_SAI1_TX_BCLK,0x6u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_23_SAI1_TX_SYNC,0x6u);
}
