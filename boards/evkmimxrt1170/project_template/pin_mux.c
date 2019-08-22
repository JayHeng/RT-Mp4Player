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
    BOARD_InitDEBUG_UART();
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitDEBUG_UART
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitDEBUG_UART(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0x10B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0x10B0u);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSDRAM
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitSDRAM(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_00_SEMC_DATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_01_SEMC_DATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_02_SEMC_DATA02, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_03_SEMC_DATA03, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_04_SEMC_DATA04, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_05_SEMC_DATA05, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_06_SEMC_DATA06, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_07_SEMC_DATA07, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_08_SEMC_DM00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_09_SEMC_ADDR00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_10_SEMC_ADDR01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_11_SEMC_ADDR02, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_12_SEMC_ADDR03, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_13_SEMC_ADDR04, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_14_SEMC_ADDR05, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_15_SEMC_ADDR06, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_16_SEMC_ADDR07, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_17_SEMC_ADDR08, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_18_SEMC_ADDR09, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_19_SEMC_ADDR11, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_20_SEMC_ADDR12, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_21_SEMC_BA0, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_22_SEMC_BA1, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_23_SEMC_ADDR10, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_24_SEMC_CAS, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_25_SEMC_RAS, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_26_SEMC_CLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_27_SEMC_CKE, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_28_SEMC_WE, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_30_SEMC_DATA08, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_31_SEMC_DATA09, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_32_SEMC_DATA10, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_33_SEMC_DATA11, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_34_SEMC_DATA12, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_35_SEMC_DATA13, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_36_SEMC_DATA14, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_37_SEMC_DATA15, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_38_SEMC_DM01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_41_SEMC_CSX00, 0U);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitCSI
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitCSI(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_CSI_PIXCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_CSI_MCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_CSI_VSYNC, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_CSI_HSYNC, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_CSI_DATA09, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_CSI_DATA08, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_CSI_DATA07, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_CSI_DATA06, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_12_CSI_DATA05, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_CSI_DATA04, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_CSI_DATA03, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_CSI_DATA02, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL, 0xD8B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA, 0xD8B0u);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLCD
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLCD(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_LCD_CLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_01_LCD_ENABLE, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_02_LCD_HSYNC, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_03_LCD_VSYNC, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_04_LCD_DATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_05_LCD_DATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_06_LCD_DATA02, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_07_LCD_DATA03, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_08_LCD_DATA04, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_09_LCD_DATA05, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_10_LCD_DATA06, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_11_LCD_DATA07, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_12_LCD_DATA08, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_13_LCD_DATA09, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_14_LCD_DATA10, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_15_LCD_DATA11, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_00_LCD_DATA12, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_01_LCD_DATA13, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_02_LCD_DATA14, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_03_LCD_DATA15, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_15_GPIO2_IO31, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_00_LCD_CLK, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_01_LCD_ENABLE, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_02_LCD_HSYNC, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_03_LCD_VSYNC, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_04_LCD_DATA00, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_05_LCD_DATA01, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_06_LCD_DATA02, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_07_LCD_DATA03, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_08_LCD_DATA04, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_09_LCD_DATA05, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_10_LCD_DATA06, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_11_LCD_DATA07, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_12_LCD_DATA08, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_13_LCD_DATA09, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_14_LCD_DATA10, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_15_LCD_DATA11, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_00_LCD_DATA12, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_01_LCD_DATA13, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_02_LCD_DATA14, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_03_LCD_DATA15, 0x01B0B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_15_GPIO2_IO31, 0x10B0u);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitCAN
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitCAN(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_14_FLEXCAN2_TX, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_15_FLEXCAN2_RX, 0U);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitENET
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitENET(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_04_ENET_RX_DATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_05_ENET_RX_DATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_06_ENET_RX_EN, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_07_ENET_TX_DATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_08_ENET_TX_DATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_09_ENET_TX_EN, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_10_ENET_REF_CLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_11_ENET_RX_ER, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_40_ENET_MDC, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_41_ENET_MDIO, 0U);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitUSDHC
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitUSDHC(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3, 0U);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitHyperFlash
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitHyperFlash(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_00_FLEXSPIB_DATA03, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_01_FLEXSPIB_DATA02, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_FLEXSPIB_DATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_FLEXSPIB_DATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_04_FLEXSPIB_SCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_05_FLEXSPIA_DQS, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_06_FLEXSPIA_SS0_B, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_07_FLEXSPIA_SCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_08_FLEXSPIA_DATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_09_FLEXSPIA_DATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_10_FLEXSPIA_DATA02, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_11_FLEXSPIA_DATA03, 0U);
}
