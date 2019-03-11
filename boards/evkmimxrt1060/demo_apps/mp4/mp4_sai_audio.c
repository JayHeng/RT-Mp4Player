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
//#include "music.h"
#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
#include "fsl_dmamux.h"
#endif
#include "fsl_sai_edma.h"
#include "fsl_wm8960.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* SAI instance and clock */
#define DEMO_CODEC_WM8960
#define DEMO_SAI SAI1
#define DEMO_SAI_CHANNEL (0)
#define DEMO_SAI_IRQ SAI1_IRQn
#define SAI_UserIRQHandler SAI1_IRQHandler

/* IRQ */
#define DEMO_SAI_TX_IRQ SAI1_IRQn
#define DEMO_SAI_RX_IRQ SAI1_IRQn

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)

#define MAX_SAI_CLK_SRC_PRE_DIV (7)
#define MAX_SAI_CLK_SRC_DIV (63)

#define MAX_SAI_BIT_CLK_DIV (7)

/* I2C instance and clock */
#define DEMO_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define DEMO_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (DEMO_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

/* DMA */
#define EXAMPLE_DMA DMA0
#define EXAMPLE_DMAMUX DMAMUX
#define EXAMPLE_TX_CHANNEL (0U)
#define EXAMPLE_RX_CHANNEL (1U)
#define EXAMPLE_SAI_TX_SOURCE kDmaRequestMuxSai1Tx
#define EXAMPLE_SAI_RX_SOURCE kDmaRequestMuxSai1Rx

#define OVER_SAMPLE_RATE (384U)
#define BUFFER_SIZE (4096)
#define BUFFER_NUM (1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);
static void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);

static uint32_t get_sai_clock_freq(void);
static void set_sai_clock_dividers(uint32_t bitWidth, uint32_t sampleRate_Hz, sai_mono_stereo_t stereo);

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t txHandle) = {0};
edma_handle_t dmaTxHandle = {0};
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t rxHandle) = {0};
edma_handle_t dmaRxHandle = {0};
sai_transfer_format_t format = {0};
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t audioBuff[BUFFER_SIZE * BUFFER_NUM], 4);
codec_handle_t codecHandle = {0};
extern codec_config_t boardCodecConfig;
volatile bool istxFinished = true;
volatile bool isrxFinished = false;
volatile uint32_t beginCount = 0;
volatile uint32_t sendCount = 0;
volatile uint32_t receiveCount = 0;
volatile uint32_t fullBlock = 0;
volatile uint32_t emptyBlock = BUFFER_NUM;

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

//#define BLOCK_SIZE (2304*2)
//#define BLOCK_NUM (2)
//
//uint8_t audio_buf[BLOCK_SIZE*BLOCK_NUM];
//int buf_index = 0;
//uint8_t audio_buf_dummy[BLOCK_SIZE] = {0};
//
//void SAI_send_audio(uint8_t * buf, uint32_t size)
//{
//
//}
//static void tx_send_dummy(void)
//{
//    sai_transfer_t xfer = {0};
//    xfer.data           = audio_buf_dummy;
//    xfer.dataSize       = BLOCK_SIZE;
//    SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
//    SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
//}

//uint8_t buf_decode[2304*2];
//uint8_t mp3_decode_one_frame(uint8_t * buf_out);
//
//static int flag_sai_tx = 0;
static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
   // flag_sai_tx = 1;
    sendCount++;
    emptyBlock++;

    if (sendCount == beginCount)
    {
        istxFinished = true;
        SAI_TransferTerminateSendEDMA(base, handle);
        sendCount = 0;
    }

}
//void task_audio_tx(void)
//{
//    if(flag_sai_tx)
//    {
//        flag_sai_tx = 0;
//        uint8_t * buf;
//        buf = audio_buf + buf_index*BLOCK_SIZE;
//        buf_index ^= 1;
//        mp3_decode_one_frame(buf);
//        sai_transfer_t xfer;
//        xfer.data           = buf;
//        xfer.dataSize       = BLOCK_SIZE;
//        SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
//    }
//}
static void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    receiveCount++;
    fullBlock++;

    if (receiveCount == beginCount)
    {
        isrxFinished = true;
        SAI_TransferTerminateReceiveEDMA(base, handle);
        receiveCount = 0;
    }
}

void sai_audio_play(uint8_t *audioData, uint32_t audioBytes)
{
    sai_transfer_t xfer = {0};
    uint32_t totalNum = 0;

    /* Wait for the send finished */
    while (istxFinished != true)
    {
    }

    /* Clear the status */
    istxFinished = false;
    sendCount = 0;

    /* Send times according to audio bytes need to play */
    beginCount = 1;

    /* Reset SAI Tx internal logic */
    SAI_TxSoftwareReset(DEMO_SAI, kSAI_ResetTypeSoftware);
    /* Do the play */
    xfer.dataSize = audioBytes;

    while (totalNum < beginCount)
    {
        xfer.data = audioData + totalNum * audioBytes;
        /* Shall make sure the sai buffer queue is not full */
        if (SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer) == kStatus_Success)
        {
            totalNum++;
        }
    }
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
    CLOCK_SetMux(kCLOCK_Lpi2cMux, DEMO_LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, DEMO_LPI2C_CLOCK_SOURCE_DIVIDER);

    // Find best dividers for SAI clock setting
    set_sai_clock_dividers(bitWidth, sampleRate_Hz, stereo);

    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
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
    EDMA_Init(EXAMPLE_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaTxHandle, EXAMPLE_DMA, EXAMPLE_TX_CHANNEL);

    DMAMUX_Init(EXAMPLE_DMAMUX);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL, (uint8_t)EXAMPLE_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL);

    /* Init SAI module */
    /*
     * config.masterSlave = kSAI_Master;
     * config.mclkSource = kSAI_MclkSourceSysclk;
     * config.protocol = kSAI_BusLeftJustified;
     * config.syncMode = kSAI_ModeAsync;
     * config.mclkOutputEnable = true;
     */
    SAI_TxGetDefaultConfig(&config);
    SAI_TxInit(DEMO_SAI, &config);

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

    SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &txHandle, txCallback, NULL, &dmaTxHandle);

    mclkSourceClockHz = get_sai_clock_freq();
    SAI_TransferTxSetFormatEDMA(DEMO_SAI, &txHandle, &format, mclkSourceClockHz, format.masterClockHz);

    /* Enable interrupt to handle FIFO error */
    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    EnableIRQ(DEMO_SAI_TX_IRQ);

    //sai_audio_play(music, sizeof(music));

    // mp3_play_song("tma.mp3");
    // tx_send_dummy();
    // task_audio_tx();
}

void SAI_UserTxIRQHandler(void)
{
    /* Clear the FEF flag */
    SAI_TxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_TxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void SAI_UserRxIRQHandler(void)
{
    SAI_RxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_RxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void SAI_UserIRQHandler(void)
{
    if (DEMO_SAI->TCSR & kSAI_FIFOErrorFlag)
    {
        SAI_UserTxIRQHandler();
    }

    if (DEMO_SAI->RCSR & kSAI_FIFOErrorFlag)
    {
        SAI_UserRxIRQHandler();
    }
}
