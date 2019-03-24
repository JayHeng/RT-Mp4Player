/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "music.h"
#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
#include "fsl_dmamux.h"
#endif
#include "fsl_sai_edma.h"
#include "fsl_wm8960.h"
#include "mp4.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* SAI instance and clock */
#define APP_CODEC_WM8960
#define APP_SAI            SAI1
#define APP_SAI_CHANNEL    (0)
#define APP_SAI_IRQ        SAI1_IRQn
#define APP_SAI_TX_IRQ     SAI1_IRQn
#define SAI_UserIRQHandler SAI1_IRQHandler
#define OVER_SAMPLE_RATE   (384U)

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define APP_SAI1_CLOCK_SOURCE_SELECT (2U)
#define MAX_SAI_CLK_SRC_PRE_DIV      (7)
#define MAX_SAI_CLK_SRC_DIV          (63)
#define MAX_SAI_BIT_CLK_DIV          (7)

/* I2C instance and clock */
#define APP_I2C LPI2C1
/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define APP_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define APP_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define APP_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (APP_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

/* DMA */
#define APP_DMA           DMA0
#define APP_DMAMUX        DMAMUX
#define APP_TX_CHANNEL    (0U)
#define APP_SAI_TX_SOURCE kDmaRequestMuxSai1Tx

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);
static uint32_t get_sai_clock_freq(void);
static void set_sai_clock_dividers(uint32_t bitWidth, uint32_t sampleRate_Hz, sai_mono_stereo_t stereo);

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t txHandle) = {0};
edma_handle_t dmaTxHandle = {0};
sai_transfer_format_t format = {0};
codec_handle_t codecHandle = {0};
extern codec_config_t boardCodecConfig;
volatile uint8_t s_txBufferQueueIndex = 0;

static uint32_t s_clockSourcePreDivider = 0;
static uint32_t s_clockSourceDivider = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 77/100)
 *                              = 786.48 MHz
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator = 77,    /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};

void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}

static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_RxError == status)
    {
    }
    else
    {
        s_txBufferQueueIndex--;
    }
}

#if MP4_SAI_TIME_ENABLE
extern void time_measure_start(void);
extern uint64_t time_measure_done(void);
#define SAI_MEASURE_TRANS 100
static sai_measure_context_t s_saiMeasureContext[SAI_MEASURE_TRANS];
static uint32_t s_saiMeasureIndex = 0;
static uint32_t s_saiTransIndex = 0;
#endif //#if MP4_SAI_TIME_ENABLE

void sai_audio_play(uint8_t *audioData, uint32_t audioBytes)
{
#if MP4_FF_TIME_ENABLE
    return;
#endif

    sai_transfer_t xfer = {0};

    while (s_txBufferQueueIndex >= AUDIO_BUFFER_QUEUE - 1)
    {
    }

#if MP4_SAI_TIME_ENABLE
    if (s_saiMeasureIndex < SAI_MEASURE_TRANS)
    {
        if (s_saiMeasureIndex)
        {
            uint64_t costTime_ns = time_measure_done();
            uint32_t transSamples = audioBytes / sizeof(AUDIO_CONV_SIZE) / AUDIO_CONV_CHANNEL;
            uint64_t expectedTime_ns = (uint64_t)transSamples * 1000000000 / AUDIO_SAMP_RATE;
            if ((costTime_ns > expectedTime_ns) && ((costTime_ns - expectedTime_ns) > (AUDIO_FRAME_ERR_NS * transSamples / AUDIO_FRAME_SIZE)))
            {
                // Note: Only record those transfers which error is more than AUDIO_FRAME_ERR_NS
                s_saiMeasureContext[s_saiMeasureIndex -1].transIndex = s_saiTransIndex;
                s_saiMeasureContext[s_saiMeasureIndex -1].costTimeSai_ns = costTime_ns;
            }
            else
            {
                s_saiMeasureIndex--;
            }
        }
        time_measure_start();
        s_saiMeasureContext[s_saiMeasureIndex].costBytes = audioBytes;
        s_saiMeasureIndex++;
    }
    s_saiTransIndex++;
#endif //#if MP4_SAI_TIME_ENABLE

    s_txBufferQueueIndex++;

    /* Do the play */
    xfer.data = audioData;
    xfer.dataSize = audioBytes;
    /* Shall make sure the sai buffer queue is not full */
    while (SAI_TransferSendEDMA(APP_SAI, &txHandle, &xfer) != kStatus_Success);
}

void set_sai_clock_dividers(uint32_t bitWidth, uint32_t sampleRate_Hz, sai_mono_stereo_t stereo)
{
    uint32_t destBitClockHz = sampleRate_Hz;
    // It is the root cause of workaround in config_sai()
    destBitClockHz *= (stereo == kSAI_Stereo) ? 2 : 1;
    if (bitWidth <= kSAI_WordWidth8bits)
    {
        destBitClockHz *= 16;
    }
    else if (bitWidth <= kSAI_WordWidth16bits)
    {
        destBitClockHz *= 32;
    }
    else if (bitWidth <= kSAI_WordWidth32bits)
    {
        destBitClockHz *= 64;
    }

    uint32_t bestPreDivider = 0;
    uint32_t bestDivider = 0;
    double minError = 1;

    for (s_clockSourcePreDivider = 0; s_clockSourcePreDivider <= MAX_SAI_CLK_SRC_PRE_DIV; s_clockSourcePreDivider++)
    {
        for (s_clockSourceDivider = 0; s_clockSourceDivider <= MAX_SAI_CLK_SRC_DIV; s_clockSourceDivider++)
        {
            uint32_t masterClockHz = get_sai_clock_freq();
            for (uint32_t bitDiv = 0; bitDiv <= MAX_SAI_BIT_CLK_DIV; bitDiv++)
            {
                uint32_t srcBitClockHz = masterClockHz / 2 / bitDiv;
                if (destBitClockHz == srcBitClockHz)
                {
                    bestPreDivider = s_clockSourcePreDivider;
                    bestDivider = s_clockSourceDivider;
                    break;
                }
                else
                {
                    double error = (destBitClockHz > srcBitClockHz) ? (destBitClockHz - srcBitClockHz) : (srcBitClockHz - destBitClockHz);
                    error /= destBitClockHz;
                    if (error < minError)
                    {
                        minError = error;
                        bestPreDivider = s_clockSourcePreDivider;
                        bestDivider = s_clockSourceDivider;
                    }
                }
            }
        }
    }

    s_clockSourcePreDivider = bestPreDivider;
    s_clockSourceDivider = bestDivider;
}

uint32_t get_sai_clock_freq(void)
{
    return (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (s_clockSourceDivider + 1U) / (s_clockSourcePreDivider + 1U));
}

/*!
 * @brief config_sai function
 */
void config_sai(uint32_t bitWidth, uint32_t sampleRate_Hz, sai_mono_stereo_t stereo)
{
    sai_config_t config;
    uint32_t mclkSourceClockHz = 0U;
    edma_config_t dmaConfig = {0};
    CLOCK_InitAudioPll(&audioPllConfig);

    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, APP_LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, APP_LPI2C_CLOCK_SOURCE_DIVIDER);

    // Find best dividers for SAI clock setting
    set_sai_clock_dividers(bitWidth, sampleRate_Hz, stereo);

    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, APP_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, s_clockSourcePreDivider);
    CLOCK_SetDiv(kCLOCK_Sai1Div, s_clockSourceDivider);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);
    BOARD_Codec_I2C_Init();

    PRINTF("SAI Configuration started!\n\r");

    /* Create EDMA handle */
    /*
     * dmaConfig.enableRoundRobinArbitration = false;
     * dmaConfig.enableHaltOnError = true;
     * dmaConfig.enableContinuousLinkMode = false;
     * dmaConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(APP_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaTxHandle, APP_DMA, APP_TX_CHANNEL);

    DMAMUX_Init(APP_DMAMUX);
    DMAMUX_SetSource(APP_DMAMUX, APP_TX_CHANNEL, (uint8_t)APP_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(APP_DMAMUX, APP_TX_CHANNEL);

    /* Init SAI module */
    /*
     * config.masterSlave = kSAI_Master;
     * config.mclkSource = kSAI_MclkSourceSysclk;
     * config.protocol = kSAI_BusLeftJustified;
     * config.syncMode = kSAI_ModeAsync;
     * config.mclkOutputEnable = true;
     */
    SAI_TxGetDefaultConfig(&config);
    SAI_TxInit(APP_SAI, &config);

    /* Configure the audio format */
    format.bitWidth = bitWidth;
    format.channel = 0U;
    // Note: this is workaround for SDK SAI driver
    if (stereo == kSAI_Stereo)
    {
        format.sampleRate_Hz = sampleRate_Hz;
    }
    else
    {
        format.sampleRate_Hz = sampleRate_Hz * 2;
    }
#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) || \
    (defined FSL_FEATURE_PCC_HAS_SAI_DIVIDER && FSL_FEATURE_PCC_HAS_SAI_DIVIDER)
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
#else
    format.masterClockHz = get_sai_clock_freq();
#endif
    format.protocol = config.protocol;
    format.stereo = stereo;
    format.isFrameSyncCompact = true;
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;
#endif

    /* Use default setting to init codec */
    CODEC_Init(&codecHandle, &boardCodecConfig);
    CODEC_SetFormat(&codecHandle, format.masterClockHz, format.sampleRate_Hz, format.bitWidth);
#if defined CODEC_USER_CONFIG
    BOARD_Codec_Config(&codecHandle);
#endif

    SAI_TransferTxCreateHandleEDMA(APP_SAI, &txHandle, txCallback, NULL, &dmaTxHandle);

    mclkSourceClockHz = get_sai_clock_freq();
    SAI_TransferTxSetFormatEDMA(APP_SAI, &txHandle, &format, mclkSourceClockHz, format.masterClockHz);

    /* Enable interrupt to handle FIFO error */
    SAI_TxEnableInterrupts(APP_SAI, kSAI_FIFOErrorInterruptEnable);
    EnableIRQ(APP_SAI_TX_IRQ);

    //sai_audio_play(music, MUSIC_LEN);
    //for (uint32_t i = 0; i < 30; i++)
    //{
    //    sai_audio_play((music + i * MUSIC_LEN / 30), MUSIC_LEN / 30);
    //}
}

void SAI_UserTxIRQHandler(void)
{
    /* Clear the FEF flag */
    SAI_TxClearStatusFlags(APP_SAI, kSAI_FIFOErrorFlag);
    SAI_TxSoftwareReset(APP_SAI, kSAI_ResetTypeFIFO);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void SAI_UserIRQHandler(void)
{
    if (APP_SAI->TCSR & kSAI_FIFOErrorFlag)
    {
        SAI_UserTxIRQHandler();
    }
}
