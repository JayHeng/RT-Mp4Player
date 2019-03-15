/*
 * Copyright 2017 NXP
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
#include "mp4.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_ELCDIF LCDIF

#if VIDEO_RESOLUTION_272P
#define APP_IMG_HEIGHT 272
#define APP_IMG_WIDTH 480
#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
#elif VIDEO_RESOLUTION_720HD
#define APP_IMG_HEIGHT 800
#define APP_IMG_WIDTH 1280
#define APP_HSW 10
#define APP_HFP 70
#define APP_HBP 80
#define APP_VSW 3
#define APP_VFP 10
#define APP_VBP 10
#endif
#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN 2
/* Back light. */
#define LCD_BL_GPIO GPIO2
#define LCD_BL_GPIO_PIN 31

/* Frame buffer data alignment, for better performance, the LCDIF frame buffer should be 64B align. */
#define FRAME_BUFFER_ALIGN 64
/*
 * For better performance, three frame buffers are used in this demo.
 */
#define APP_LCD_FB_NUM 2 /* LCD frame buffer number. */
#define APP_LCD_FB_BPP 3 /* LCD frame buffer byte per pixel, RGB888 format, 24-bit. */

/* Cache line size. */
#ifndef FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#define FSL_FEATURE_L2CACHE_LINESIZE_BYTE 0
#endif
#ifndef FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#define FSL_FEATURE_L1DCACHE_LINESIZE_BYTE 0
#endif

#if (FSL_FEATURE_L2CACHE_LINESIZE_BYTE > FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#define APP_CACHE_LINE_SIZE FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#else
#define APP_CACHE_LINE_SIZE FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#endif

#define APP_PXP PXP
#if VIDEO_RESOLUTION_272P
#define APP_PS_WIDTH 480/* 720,352,image resolution*/
#define APP_BPP 4U
#elif VIDEO_RESOLUTION_720HD
#define APP_PS_WIDTH 1280/* 1280,800,image resolution*/
#define APP_BPP 4U
#endif

#define APP_PS_ULC_X 0U
#define APP_PS_ULC_Y 0U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_EnableLcdInterrupt(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool g_lcdFramePending = false;
static void *volatile s_fbList = NULL; /* List to the frame buffers. */
static void *volatile inactiveBuf = NULL;
static void *volatile activeBuf = NULL;

static pxp_output_buffer_config_t outputBufferConfig;
static pxp_ps_buffer_config_t psBufferConfig;
AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_psBufferLcd[APP_LCD_FB_NUM][APP_IMG_HEIGHT][APP_IMG_WIDTH], FRAME_BUFFER_ALIGN);

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Enable interrupt. */
void BOARD_EnableLcdInterrupt(void)
{
    EnableIRQ(LCDIF_IRQn);
}

/* Initialize the LCD_DISP. */
void BOARD_InitLcd(void)
{
    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };

#if VIDEO_RESOLUTION_272P
    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);
    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);
    volatile uint32_t i = 0x100U;
    while (i--)
    {
    }
    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);
#elif VIDEO_RESOLUTION_720HD
    // Do nothing
#endif

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
}

#if VIDEO_RESOLUTION_272P
void BOARD_InitLcdifPixelClock(void)
{
    /*
     * The desired output frame rate is 60Hz. So the pixel clock frequency is:
     * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
     * Here set the LCDIF pixel clock to 9.3M.
     */

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 93MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 31, .postDivider = 8, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}
#elif VIDEO_RESOLUTION_720HD
void BOARD_InitLcdifPixelClock(void)
{
    /*
     * The desired output frame rate is 60Hz. So the pixel clock frequency is:
     * (1280 + 10 + 80 + 70) * (800 + 3 + 10 + 10) * 60 = 71M.
     * Here set the LCDIF pixel clock to 70.5M.
     */

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 70.5MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 47, .postDivider = 16, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 0);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 0);
}
#endif

/* Put the unused frame buffer to the s_fbList. */
static void APP_PutFrameBuffer(void *fb)
{
    *(void **)fb = s_fbList;
    s_fbList = fb;
}

static void APP_InitPxp(void)
{
    PXP_Init(APP_PXP);

    /* PS configure. */
    psBufferConfig.pixelFormat = kPXP_PsPixelFormatYVU420;
    psBufferConfig.swapByte = false;
    psBufferConfig.bufferAddr = 0U;
    psBufferConfig.bufferAddrU = 0U;
    psBufferConfig.bufferAddrV = 0U;
    psBufferConfig.pitchBytes = APP_PS_WIDTH;

    PXP_SetProcessSurfaceBackGroundColor(APP_PXP, 0U);

    PXP_SetProcessSurfaceBufferConfig(APP_PXP, &psBufferConfig);

    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(APP_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

    /* Output config. */
    outputBufferConfig.pixelFormat = kPXP_OutputPixelFormatRGB888;
    outputBufferConfig.interlacedMode = kPXP_OutputProgressive;
    outputBufferConfig.buffer0Addr = (uint32_t)s_psBufferLcd[0];
    outputBufferConfig.buffer1Addr = 0U;
    outputBufferConfig.pitchBytes = APP_IMG_WIDTH * APP_BPP;
    outputBufferConfig.width = APP_IMG_WIDTH;
    outputBufferConfig.height = APP_IMG_HEIGHT;

    PXP_SetOutputBufferConfig(APP_PXP, &outputBufferConfig);

    /* Disable CSC1, it is enabled by default. */
    PXP_SetCsc1Mode(APP_PXP, kPXP_Csc1YCbCr2RGB);
    PXP_EnableCsc1(APP_PXP, true);
}

static void APP_InitLcdif(void)
{
    const elcdif_rgb_mode_config_t config = {
        .panelWidth = APP_IMG_WIDTH,
        .panelHeight = APP_IMG_HEIGHT,
        .hsw = APP_HSW,
        .hfp = APP_HFP,
        .hbp = APP_HBP,
        .vsw = APP_VSW,
        .vfp = APP_VFP,
        .vbp = APP_VBP,
        .polarityFlags = APP_POL_FLAGS,
        .bufferAddr = (uint32_t)s_psBufferLcd[0],
        .pixelFormat = kELCDIF_PixelFormatXRGB8888,
        .dataBus = APP_LCDIF_DATA_BUS,
    };

    ELCDIF_RgbModeInit(APP_ELCDIF, &config);

#if VIDEO_RESOLUTION_272P
    // Do nothing
#elif VIDEO_RESOLUTION_720HD
    /* Update the eLCDIF AXI master features for better performance */
    APP_ELCDIF->CTRL2 = 0x00700000;
#endif

    ELCDIF_RgbModeStart(APP_ELCDIF);
}

void lcd_video_display(unsigned char *buf[], int xsize, int ysize)
{
    static uint8_t curLcdBufferIdx = 1U;
    psBufferConfig.bufferAddr = (uint32_t)buf[0];
    psBufferConfig.bufferAddrU = (uint32_t)buf[1];
    psBufferConfig.bufferAddrV = (uint32_t)buf[2];

    PXP_SetProcessSurfaceBufferConfig(APP_PXP, &psBufferConfig);
    PXP_SetProcessSurfaceScaler(APP_PXP, xsize, ysize, APP_IMG_WIDTH, APP_IMG_HEIGHT);
    PXP_SetProcessSurfacePosition(APP_PXP,
                                  APP_PS_ULC_X,
                                  APP_PS_ULC_Y,
                                  APP_PS_ULC_X + APP_IMG_WIDTH - 1U,
                                  APP_PS_ULC_Y + APP_IMG_HEIGHT - 1U);
    outputBufferConfig.buffer0Addr = (uint32_t)s_psBufferLcd[curLcdBufferIdx];
    PXP_SetOutputBufferConfig(APP_PXP, &outputBufferConfig);
    PXP_Start(APP_PXP);
    /* Wait for process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(APP_PXP)))
    {
    }
    PXP_ClearStatusFlags(APP_PXP, kPXP_CompleteFlag);
    ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)s_psBufferLcd[curLcdBufferIdx]);
    ELCDIF_ClearInterruptStatus(APP_ELCDIF, kELCDIF_CurFrameDone);
    while (!(kELCDIF_CurFrameDone & ELCDIF_GetInterruptStatus(APP_ELCDIF)))
    {
    }
    curLcdBufferIdx++;
    curLcdBufferIdx %= APP_LCD_FB_NUM;
}

/*!
 * @brief config_lcd function
 */
void config_lcd(void)
{
    BOARD_InitLcdifPixelClock();
    BOARD_InitLcd();

    ELCDIF_EnableInterrupts(APP_ELCDIF, kELCDIF_CurFrameDoneInterruptEnable);
    ELCDIF_RgbModeStart(APP_ELCDIF);

    APP_InitPxp();
    APP_InitLcdif();
    BOARD_EnableLcdInterrupt();
}

void APP_LCDIF_IRQHandler(void)
{
    uint32_t intStatus;

    intStatus = ELCDIF_GetInterruptStatus(APP_ELCDIF);

    ELCDIF_ClearInterruptStatus(APP_ELCDIF, intStatus);

    if (intStatus & kELCDIF_CurFrameDone)
    {
        if (g_lcdFramePending)
        {
            /*
             * The inactive buffer turns to be active frame buffer, the
             * old active frame buffer is not used, so put it into the
             * frame buffer list.
             */
            APP_PutFrameBuffer(activeBuf);

            activeBuf = inactiveBuf;
            g_lcdFramePending = false;
        }
    }
}

void LCDIF_IRQHandler(void)
{
    APP_LCDIF_IRQHandler();
    __DSB();
}
