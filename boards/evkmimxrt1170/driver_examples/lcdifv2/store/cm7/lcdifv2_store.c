/*
 * Copyright 2019 NXP Semiconductors, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_lcdifv2.h"
#include "lcdifv2_support.h"
#include "fsl_debug_console.h"

/*
 * In this example, four input layers are used, the input frame buffer content
 * are all 8BPP index and the values are all set to 0. Because the LUT value
 * are different for each layer, to each layer output are different.
 *
 * The LCDIF v2 stores the output frame to output buffer, the output buffer
 * values are verified after store finished.
 *
 * At the same time, the output frame are shown in the panel,
 *
 * +-------------------------------+
 * |            RED                |
 * +-------------------------------+
 * |            GREEN              |
 * +-------------------------------+
 * |            BLUE               |
 * +-------------------------------+
 * |            WHITE              |
 * +-------------------------------+
 *
 */

#include "pin_mux.h"
#include "board.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_CORE_ID 0

/* Input is 8BPP index. */
#define DEMO_INPUT_BYTE_PER_PIXEL 1U
/* Output is ARGB. */
#define DEMO_OUTPUT_BYTE_PER_PIXEL 4U

#define DEMO_IMG_HEIGHT DEMO_PANEL_HEIGHT
#define DEMO_IMG_WIDTH DEMO_PANEL_WIDTH

#define DEMO_INPUT_IMG_HEIGHT (DEMO_PANEL_HEIGHT / 4)
#define DEMO_INPUT_IMG_WIDTH (DEMO_PANEL_WIDTH)

#define DEMO_OUTPUT_IMG_HEIGHT (DEMO_PANEL_HEIGHT)
#define DEMO_OUTPUT_IMG_WIDTH (DEMO_PANEL_WIDTH)

#define DEMO_INPUT_LAYER_COUNT 4

#define DEMO_LAYER0_COLOR_ARGB 0xFFFF0000
#define DEMO_LAYER1_COLOR_ARGB 0xFF00FF00
#define DEMO_LAYER2_COLOR_ARGB 0xFF0000FF
#define DEMO_LAYER3_COLOR_ARGB 0xFFFFFFFF

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

AT_NONCACHEABLE_SECTION(static uint8_t s_inputBuffer[DEMO_INPUT_IMG_HEIGHT][DEMO_INPUT_IMG_WIDTH]);

AT_NONCACHEABLE_SECTION(static uint32_t s_outputBuffer[DEMO_OUTPUT_IMG_HEIGHT][DEMO_OUTPUT_IMG_WIDTH]);

static const uint32_t lut0[] = {DEMO_LAYER0_COLOR_ARGB};

static const uint32_t lut1[] = {DEMO_LAYER1_COLOR_ARGB};

static const uint32_t lut2[] = {DEMO_LAYER2_COLOR_ARGB};

static const uint32_t lut3[] = {DEMO_LAYER3_COLOR_ARGB};

/*******************************************************************************
 * Code
 ******************************************************************************/

void DEMO_LCDIFV2_InitDisplay(void)
{
    const lcdifv2_display_config_t lcdifv2Config = {
        .panelWidth    = DEMO_PANEL_WIDTH,
        .panelHeight   = DEMO_PANEL_HEIGHT,
        .hsw           = DEMO_HSW,
        .hfp           = DEMO_HFP,
        .hbp           = DEMO_HBP,
        .vsw           = DEMO_VSW,
        .vfp           = DEMO_VFP,
        .vbp           = DEMO_VBP,
        .polarityFlags = DEMO_POL_FLAGS,
        .lineOrder     = kLCDIFV2_LineOrderRGB,
    };

    if (kStatus_Success != BOARD_InitDisplayInterface())
    {
        PRINTF("Display interface initialize failed\r\n");

        while (1)
        {
        }
    }

    LCDIFV2_Init(DEMO_LCDIFV2);

    LCDIFV2_SetDisplayConfig(DEMO_LCDIFV2, &lcdifv2Config);
}

void DEMO_LCDIFV2_ConfigInputLayer(void)
{
    const lcdifv2_buffer_config_t fbConfig = {
        .strideBytes = DEMO_INPUT_IMG_WIDTH * DEMO_INPUT_BYTE_PER_PIXEL,
        .pixelFormat = kLCDIFV2_PixelFormatIndex8BPP,
    };

    memset(s_inputBuffer, 0, sizeof(s_inputBuffer));

    LCDIFV2_SetLut(DEMO_LCDIFV2, 0, lut0, ARRAY_SIZE(lut0), false);
    LCDIFV2_SetLut(DEMO_LCDIFV2, 1, lut1, ARRAY_SIZE(lut1), false);
    LCDIFV2_SetLut(DEMO_LCDIFV2, 2, lut2, ARRAY_SIZE(lut2), false);
    LCDIFV2_SetLut(DEMO_LCDIFV2, 3, lut3, ARRAY_SIZE(lut3), false);

    LCDIFV2_SetLayerOffset(DEMO_LCDIFV2, 0, 0, 0);
    LCDIFV2_SetLayerOffset(DEMO_LCDIFV2, 1, 0, DEMO_INPUT_IMG_HEIGHT * 1);
    LCDIFV2_SetLayerOffset(DEMO_LCDIFV2, 2, 0, DEMO_INPUT_IMG_HEIGHT * 2);
    LCDIFV2_SetLayerOffset(DEMO_LCDIFV2, 3, 0, DEMO_INPUT_IMG_HEIGHT * 3);

    for (uint8_t layer = 0; layer < DEMO_INPUT_LAYER_COUNT; layer++)
    {
        LCDIFV2_SetLayerBufferConfig(DEMO_LCDIFV2, layer, &fbConfig);

        LCDIFV2_SetLayerSize(DEMO_LCDIFV2, layer, DEMO_INPUT_IMG_WIDTH, DEMO_INPUT_IMG_HEIGHT);

        LCDIFV2_SetLayerBufferAddr(DEMO_LCDIFV2, layer, (uint32_t)s_inputBuffer);

        LCDIFV2_EnableLayer(DEMO_LCDIFV2, layer, true);

        LCDIFV2_TriggerLayerShadowLoad(DEMO_LCDIFV2, layer);
    }
}

void DEMO_LCDIFV2_Store(void)
{
    const lcdifv2_store_buffer_config_t storeConfig = {
        .bufferAddr  = (uint32_t)s_outputBuffer,
        .strideBytes = DEMO_OUTPUT_IMG_WIDTH * DEMO_OUTPUT_BYTE_PER_PIXEL,
        .pixelFormat = kLCDIFV2_StorePixelFormatARGB8888,
    };

    LCDIFV2_EnableDisplay(DEMO_LCDIFV2, true);

    LCDIFV2_SetStoreBufferConfig(DEMO_LCDIFV2, &storeConfig);

    LCDIFV2_StartStore(DEMO_LCDIFV2, false);

    while ((kLCDIFV2_StoreFrameDoneInterrupt & LCDIFV2_GetInterruptStatus(DEMO_LCDIFV2, DEMO_CORE_ID)) == 0)
    {
    }
}

void DEMO_LCDIFV2_VerifyStore(void)
{
    uint32_t row, col;

    for (row = 0; row < DEMO_INPUT_IMG_HEIGHT; row++)
    {
        for (col = 0; col < DEMO_INPUT_IMG_WIDTH; col++)
        {
            if (s_outputBuffer[row][col] != DEMO_LAYER0_COLOR_ARGB)
            {
                PRINTF("Error at row %d col %d\r\n", row, col);
                while (1)
                    ;
            }
        }
    }

    for (; row < DEMO_INPUT_IMG_HEIGHT * 2; row++)
    {
        for (col = 0; col < DEMO_INPUT_IMG_WIDTH; col++)
        {
            if (s_outputBuffer[row][col] != DEMO_LAYER1_COLOR_ARGB)
            {
                PRINTF("Error at row %d col %d\r\n", row, col);
                while (1)
                    ;
            }
        }
    }

    for (; row < DEMO_INPUT_IMG_HEIGHT * 3; row++)
    {
        for (col = 0; col < DEMO_INPUT_IMG_WIDTH; col++)
        {
            if (s_outputBuffer[row][col] != DEMO_LAYER2_COLOR_ARGB)
            {
                PRINTF("Error at row %d col %d\r\n", row, col);
                while (1)
                    ;
            }
        }
    }

    for (; row < DEMO_INPUT_IMG_HEIGHT * 4; row++)
    {
        for (col = 0; col < DEMO_INPUT_IMG_WIDTH; col++)
        {
            if (s_outputBuffer[row][col] != DEMO_LAYER3_COLOR_ARGB)
            {
                PRINTF("Error at row %d col %d\r\n", row, col);
                while (1)
                    ;
            }
        }
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_BootClockRUN();
    BOARD_InitLpuartPins();
    BOARD_InitMipiPanelPins();
    BOARD_InitDebugConsole();
    BOARD_InitLcdifClock();

    PRINTF("LCDIF v2 store example start...\r\n");

    DEMO_LCDIFV2_InitDisplay();

    DEMO_LCDIFV2_ConfigInputLayer();

    DEMO_LCDIFV2_Store();

    DEMO_LCDIFV2_VerifyStore();

    PRINTF("LCDIF v2 store example success...\r\n");

    while (1)
    {
    }
}
