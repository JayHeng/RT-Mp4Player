/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_gpio.h"
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
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_25_LPUART1_RXD, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0x0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0x0u);
}

void BOARD_InitMipiPanelPins(void)
{
    const gpio_pin_config_t DISP_BL_config = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0U, .interruptMode = kGPIO_NoIntmode};

    GPIO_PinInit(GPIO9, 29U, &DISP_BL_config);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_30_GPIO9_IO29, 0U);

    const gpio_pin_config_t DISP_RST_config = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0U, .interruptMode = kGPIO_NoIntmode};

    GPIO_PinInit(GPIO9, 1U, &DISP_RST_config);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_02_GPIO9_IO01, 0U);

    const gpio_pin_config_t DISP_POWER_config = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0U, .interruptMode = kGPIO_NoIntmode};

    GPIO_PinInit(GPIO11, 16U, &DISP_POWER_config);

    IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_15_GPIO11_IO16, 0U);
}

void BOARD_InitMipiCameraPins(void)
{
    const gpio_pin_config_t CAMERA_PWDN_config = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0U, .interruptMode = kGPIO_NoIntmode};

    GPIO_PinInit(GPIO9, 25U, &CAMERA_PWDN_config);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_26_GPIO9_IO25, 0U);

    const gpio_pin_config_t CAMERA_RST_config = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0U, .interruptMode = kGPIO_NoIntmode};

    GPIO_PinInit(GPIO11, 15U, &CAMERA_RST_config);

    IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_14_GPIO11_IO15, 0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_LPSR_06_LPI2C6_SDA, 1U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_LPSR_07_LPI2C6_SCL, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_LPSR_06_LPI2C6_SDA, 0xD8B0u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_LPSR_07_LPI2C6_SCL, 0xD8B0u);
}
