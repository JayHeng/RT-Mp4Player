/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_pxp.h"
#include "fsl_elcdif.h"
#include "fsl_cache.h"
#include "fsl_gpio.h"
#include "fsl_camera.h"
#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"
#include "fsl_csi.h"
#include "fsl_csi_camera_adapter.h"
#include "fsl_ov7725.h"
#include "fsl_mt9m114.h"
#include "mp4.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Camera definition. */
#define APP_CAMERA_HEIGHT 272
#define APP_CAMERA_WIDTH 480
#define APP_CAMERA_CONTROL_FLAGS (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge)

#define APP_CAMERA_OV7725 0
#define APP_CAMERA_MT9M114 1

#define APP_CAMERA_TYPE APP_CAMERA_MT9M114

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void CSI_DriverIRQHandler(void);
extern void pinmux_i2c_init(void);
extern void pinmux_csi_init(void);
static void BOARD_PullCameraResetPin(bool pullUp);

/* OV7725 connect to CSI. */
static csi_resource_t csiResource = {
    .csiBase = CSI,
};

static csi_private_data_t csiPrivateData;

camera_receiver_handle_t g_cameraReceiver = {
    .resource    = &csiResource,
    .ops         = &csi_ops,
    .privateData = &csiPrivateData,
};

extern camera_receiver_handle_t g_cameraReceiver;

#if (APP_CAMERA_TYPE == APP_CAMERA_OV7725)
static void BOARD_PullCameraPowerDownPin(bool pullUp);
static ov7725_resource_t ov7725Resource = {
    .i2cSendFunc       = BOARD_Camera_I2C_SendSCCB,
    .i2cReceiveFunc    = BOARD_Camera_I2C_ReceiveSCCB,
    .pullResetPin      = BOARD_PullCameraResetPin,
    .pullPowerDownPin  = BOARD_PullCameraPowerDownPin,
    .inputClockFreq_Hz = 24000000,
};

camera_device_handle_t g_cameraDevice = {
    .resource = &ov7725Resource,
    .ops      = &ov7725_ops,
};
#else
static mt9m114_resource_t mt9m114Resource = {
    .i2cSendFunc       = BOARD_Camera_I2C_Send,
    .i2cReceiveFunc    = BOARD_Camera_I2C_Receive,
    .pullResetPin      = BOARD_PullCameraResetPin,
    .inputClockFreq_Hz = 24000000,
};

camera_device_handle_t g_cameraDevice = {
    .resource = &mt9m114Resource,
    .ops      = &mt9m114_ops,
};
#endif

extern camera_device_handle_t g_cameraDevice;

/*******************************************************************************
 * Variables
 ******************************************************************************/

camera_csi_cfg_t g_cameraCsiCfg;

extern uint8_t g_psBufferLcd[APP_LCD_FB_NUM][APP_IMG_HEIGHT][APP_IMG_WIDTH][APP_BPP];

/*******************************************************************************
 * Code
 ******************************************************************************/

static void BOARD_PullCameraResetPin(bool pullUp)
{
    /* Reset pin is connected to DCDC_3V3. */
    return;
}

#if (APP_CAMERA_TYPE == APP_CAMERA_OV7725)
static void BOARD_PullCameraPowerDownPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(GPIO1, 4, 1);
    }
    else
    {
        GPIO_PinWrite(GPIO1, 4, 0);
    }
}
#else
/*
 * MT9M114 camera module has PWDN pin, but the pin is not
 * connected internally, MT9M114 does not have power down pin.
 * The reset pin is connected to high, so the module could
 * not be reseted, so at the begining, use GPIO to let camera
 * release the I2C bus.
 */
static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 0x200; i++)
    {
        __NOP();
    }
}

#define CAMERA_I2C_SCL_GPIO GPIO1
#define CAMERA_I2C_SCL_PIN 16
#define CAMERA_I2C_SDA_GPIO GPIO1
#define CAMERA_I2C_SDA_PIN 17

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i                          = 0;
    const gpio_pin_config_t pin_config = {.direction = kGPIO_DigitalOutput, .outputLogic = 1};

    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0U);

    GPIO_PinInit(CAMERA_I2C_SCL_GPIO, CAMERA_I2C_SCL_PIN, &pin_config);
    GPIO_PinInit(CAMERA_I2C_SDA_GPIO, CAMERA_I2C_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(CAMERA_I2C_SDA_GPIO, CAMERA_I2C_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(CAMERA_I2C_SCL_GPIO, CAMERA_I2C_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(CAMERA_I2C_SDA_GPIO, CAMERA_I2C_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(CAMERA_I2C_SCL_GPIO, CAMERA_I2C_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(CAMERA_I2C_SCL_GPIO, CAMERA_I2C_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(CAMERA_I2C_SDA_GPIO, CAMERA_I2C_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(CAMERA_I2C_SCL_GPIO, CAMERA_I2C_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(CAMERA_I2C_SDA_GPIO, CAMERA_I2C_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
#endif

void BOARD_InitCameraResource(void)
{
    BOARD_Camera_I2C_Init();

    /* CSI MCLK select 24M. */
    /*
     * CSI clock source:
     *
     * 00 derive clock from osc_clk (24M)
     * 01 derive clock from PLL2 PFD2
     * 10 derive clock from pll3_120M
     * 11 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_CsiMux, 0);
    /*
     * CSI clock divider:
     *
     * 000 divide by 1
     * 001 divide by 2
     * 010 divide by 3
     * 011 divide by 4
     * 100 divide by 5
     * 101 divide by 6
     * 110 divide by 7
     * 111 divide by 8
     */
    CLOCK_SetDiv(kCLOCK_CsiDiv, 0);

    /*
     * For RT1060, there is not dedicate clock gate for CSI MCLK, it use CSI
     * clock gate.
     */

    /* Set the pins for CSI reset and power down. */
    gpio_pin_config_t pinConfig = {
        kGPIO_DigitalOutput,
        1,
    };

    GPIO_PinInit(GPIO1, 4, &pinConfig);
}

void csi_camera_capture(uint32_t *activeBufferAddr, uint32_t *inactiveBufferAddr)
{
    CAMERA_RECEIVER_SubmitEmptyBuffer(&g_cameraReceiver, *inactiveBufferAddr);

    /* Wait to get the full frame buffer to show. */
    while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&g_cameraReceiver, activeBufferAddr))
    {
    }
}

/*!
 * @brief config_csi function
 */
uint32_t config_csi(camera_csi_cfg_t *csiCfg)
{
    uint32_t dummyFrameAddr[APP_LCD_FB_NUM - 1];

#if (APP_CAMERA_TYPE != APP_CAMERA_OV7725)
    BOARD_I2C_ReleaseBus();
#endif

    pinmux_i2c_init();
    pinmux_csi_init();

    BOARD_InitCameraResource();

    const camera_config_t cameraConfig = {
        .pixelFormat                = kVIDEO_PixelFormatRGB565,
        .bytesPerPixel              = APP_BPP,
        .resolution                 = FSL_VIDEO_RESOLUTION(APP_CAMERA_WIDTH, APP_CAMERA_HEIGHT),
        .frameBufferLinePitch_Bytes = APP_IMG_WIDTH * APP_BPP,
        .interface                  = kCAMERA_InterfaceGatedClock,
        .controlFlags               = APP_CAMERA_CONTROL_FLAGS,
        .framePerSec                = 30,
    };

    CAMERA_RECEIVER_Init(&g_cameraReceiver, &cameraConfig, NULL, NULL);
    CAMERA_DEVICE_Init(&g_cameraDevice, &cameraConfig);
    CAMERA_DEVICE_Start(&g_cameraDevice);

    // Submit the empty frame buffers to buffer queue.
    for (uint32_t i = 0; i < APP_LCD_FB_NUM; i++)
    {
        CAMERA_RECEIVER_SubmitEmptyBuffer(&g_cameraReceiver, (uint32_t)(g_psBufferLcd[i]));
    }

    CAMERA_RECEIVER_Start(&g_cameraReceiver);

    // Cache (APP_LCD_FB_NUM - 1) frames.
    for (uint32_t i = 0; i < APP_LCD_FB_NUM - 1; i++)
    {
        while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&g_cameraReceiver, &dummyFrameAddr[i]))
        {
        }
    }

    // Find out the last empty frame buffer address and return it.
    for (uint32_t i = 0; i < APP_LCD_FB_NUM; i++)
    {
        uint32_t j;
        for (j = 0; j < APP_LCD_FB_NUM - 1; j++)
        {
            if (dummyFrameAddr[j] == (uint32_t)(g_psBufferLcd[i]))
            {
                break;
            }
        }
        if (j == APP_LCD_FB_NUM - 1)
        {
            return (uint32_t)(g_psBufferLcd[i]);
        }
    }
}

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    __DSB();
}

