/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v8.0
processor: MIMXRT1176xxxxx
package_id: MIMXRT1176DVMAA
mcu_data: ksdk2_0
processor_version: 0.8.1
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
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
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
    BOARD_InitLPUART();
    BOARD_InitSDCARD();
    BOARD_InitMIPI();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitLPUART:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: M15, peripheral: LPUART1, signal: RXD, pin_signal: GPIO_AD_25, software_input_on: Enable}
  - {pin_num: L13, peripheral: LPUART1, signal: TXD, pin_signal: GPIO_AD_24}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLPUART, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLPUART(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,          /* GPIO_AD_24 is configured as LPUART1_TXD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,          /* GPIO_AD_25 is configured as LPUART1_RXD */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_25 */
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitSDCARD:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: K16, peripheral: CM7_GPIO3, signal: 'gpio_mux_io_cm7, 31', pin_signal: GPIO_AD_32}
  - {pin_num: J16, peripheral: USDHC1, signal: usdhc_vselect, pin_signal: GPIO_AD_34}
  - {pin_num: B16, peripheral: USDHC1, signal: usdhc_cmd, pin_signal: GPIO_SD_B1_00, software_input_on: Enable, pull_down_pull_up_config: Pull_Up, pdrv_config: High_Driver}
  - {pin_num: D15, peripheral: USDHC1, signal: usdhc_clk, pin_signal: GPIO_SD_B1_01, software_input_on: Enable, pull_down_pull_up_config: No_Pull}
  - {pin_num: C15, peripheral: USDHC1, signal: 'usdhc_data, 0', pin_signal: GPIO_SD_B1_02, software_input_on: Enable, pdrv_config: High_Driver}
  - {pin_num: B17, peripheral: USDHC1, signal: 'usdhc_data, 1', pin_signal: GPIO_SD_B1_03, software_input_on: Enable, pdrv_config: High_Driver}
  - {pin_num: B15, peripheral: USDHC1, signal: 'usdhc_data, 2', pin_signal: GPIO_SD_B1_04, software_input_on: Enable, pdrv_config: High_Driver}
  - {pin_num: A16, peripheral: USDHC1, signal: 'usdhc_data, 3', pin_signal: GPIO_SD_B1_05, software_input_on: Enable, pull_down_pull_up_config: Pull_Up, pdrv_config: High_Driver}
  - {pin_num: G17, peripheral: GPIO10, signal: 'gpio_io, 02', pin_signal: GPIO_AD_35}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSDCARD, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitSDCARD(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_32_GPIO_MUX3_IO31,       /* GPIO_AD_32 is configured as GPIO_MUX3_IO31 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_34_USDHC1_VSELECT,       /* GPIO_AD_34 is configured as USDHC1_VSELECT */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_35_GPIO10_IO02,          /* GPIO_AD_35 is configured as GPIO10_IO02 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_00_USDHC1_CMD,        /* GPIO_SD_B1_00 is configured as USDHC1_CMD */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_00 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_01_USDHC1_CLK,        /* GPIO_SD_B1_01 is configured as USDHC1_CLK */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_01 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_02_USDHC1_DATA0,      /* GPIO_SD_B1_02 is configured as USDHC1_DATA0 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_02 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_03_USDHC1_DATA1,      /* GPIO_SD_B1_03 is configured as USDHC1_DATA1 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_03 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_04_USDHC1_DATA2,      /* GPIO_SD_B1_04 is configured as USDHC1_DATA2 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_04 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_05_USDHC1_DATA3,      /* GPIO_SD_B1_05 is configured as USDHC1_DATA3 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_05 */
  IOMUXC_GPR->GPR43 = ((IOMUXC_GPR->GPR43 &
    (~(IOMUXC_GPR_GPR43_GPIO_MUX3_GPIO_SEL_HIGH_MASK))) /* Mask bits to zero which are setting */
      | IOMUXC_GPR_GPR43_GPIO_MUX3_GPIO_SEL_HIGH(0x8000U) /* GPIO3 and CM7_GPIO3 share same IO MUX function, GPIO_MUX3 selects one GPIO function: 0x8000U */
    );
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_00_USDHC1_CMD,        /* GPIO_SD_B1_00 PAD functional properties : */
      0x04U);                                 /* PDRV Field: high drive strength
                                                 Pull Down Pull Up Field: Internal pullup resistor enabled
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_01_USDHC1_CLK,        /* GPIO_SD_B1_01 PAD functional properties : */
      0x0CU);                                 /* PDRV Field: high drive strength
                                                 Pull Down Pull Up Field: No Pull
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_02_USDHC1_DATA0,      /* GPIO_SD_B1_02 PAD functional properties : */
      0x04U);                                 /* PDRV Field: high drive strength
                                                 Pull Down Pull Up Field: Internal pullup resistor enabled
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_03_USDHC1_DATA1,      /* GPIO_SD_B1_03 PAD functional properties : */
      0x04U);                                 /* PDRV Field: high drive strength
                                                 Pull Down Pull Up Field: Internal pullup resistor enabled
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_04_USDHC1_DATA2,      /* GPIO_SD_B1_04 PAD functional properties : */
      0x04U);                                 /* PDRV Field: high drive strength
                                                 Pull Down Pull Up Field: Internal pullup resistor enabled
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_05_USDHC1_DATA3,      /* GPIO_SD_B1_05 PAD functional properties : */
      0x04U);                                 /* PDRV Field: high drive strength
                                                 Pull Down Pull Up Field: Internal pullup resistor enabled
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitMIPI:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: R13, peripheral: GPIO9, signal: 'gpio_io, 01', pin_signal: GPIO_AD_02}
  - {pin_num: K17, peripheral: GPIO9, signal: 'gpio_io, 29', pin_signal: GPIO_AD_30}
  - {pin_num: A4, peripheral: GPIO11, signal: 'gpio_io, 16', pin_signal: GPIO_DISP_B2_15}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitMIPI, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitMIPI(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_02_GPIO9_IO01,           /* GPIO_AD_02 is configured as GPIO9_IO01 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_30_GPIO9_IO29,           /* GPIO_AD_30 is configured as GPIO9_IO29 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_15_GPIO11_IO16,     /* GPIO_DISP_B2_15 is configured as GPIO11_IO16 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
