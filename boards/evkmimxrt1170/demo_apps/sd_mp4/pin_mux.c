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

void BOARD_InitLPUART(void)
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_25_LPUART1_RXD, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0x0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0x0u);
}

void BOARD_InitSDCARD(void)
{
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

void BOARD_InitMIPI(void)
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_30_GPIO9_IO29, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_02_GPIO9_IO01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_15_GPIO11_IO16, 0U);
}

void BOARD_InitSAI(void)
{
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

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);
    BOARD_InitLPUART();
    BOARD_InitSDCARD();
    BOARD_InitMIPI();
    BOARD_InitSAI();
}
