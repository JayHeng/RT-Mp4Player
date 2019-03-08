/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_elcdif.h"
#include "fsl_cache.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "board.h"
#include "fsl_pxp.h"
#include "fsl_sai.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_ELCDIF LCDIF

#define APP_IMG_HEIGHT 272
#define APP_IMG_WIDTH 480
#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
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
#define APP_LCD_FB_NUM 3 /* LCD frame buffer number. */
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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_EnableLcdInterrupt(void);
static status_t sdcardWaitCardInsert(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static volatile bool g_lcdFramePending = false;
static void *volatile s_fbList = NULL; /* List to the frame buffers. */
static void *volatile inactiveBuf = NULL;
static void *volatile activeBuf = NULL;


SDK_ALIGN(static uint8_t g_frameBuffer[APP_LCD_FB_NUM][SDK_SIZEALIGN(APP_IMG_HEIGHT * APP_IMG_WIDTH * APP_LCD_FB_BPP,
                                                                     APP_CACHE_LINE_SIZE)],
          APP_CACHE_LINE_SIZE);
AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */
AT_NONCACHEABLE_SECTION(static FIL jpgFil);

static const sdmmchost_detect_card_t s_sdCardDetect = {
#ifndef BOARD_SD_DETECT_TYPE
    .cdType = kSDMMCHOST_DetectCardByGpioCD,
#else
    .cdType = BOARD_SD_DETECT_TYPE,
#endif
    .cdTimeOut_ms = (~0U),
};
/*! @brief SDMMC card power control configuration */
#if defined DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
static const sdmmchost_pwr_card_t s_sdCardPwrCtrl = {
    .powerOn = BOARD_PowerOnSDCARD, .powerOnDelay_ms = 500U, .powerOff = BOARD_PowerOffSDCARD, .powerOffDelay_ms = 0U,
};
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/
extern void APP_LCDIF_IRQHandler(void);

void LCDIF_IRQHandler(void)
{
    APP_LCDIF_IRQHandler();
}

/* Enable interrupt. */
void BOARD_EnableLcdInterrupt(void)
{
    EnableIRQ(LCDIF_IRQn);
}

/* Initialize the LCD_DISP. */
void BOARD_InitLcd(void)
{
    volatile uint32_t i = 0x100U;

    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);

    while (i--)
    {
    }

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
}

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

static void BOARD_USDHCClockConfiguration(void)
{
    /*configure system pll PFD2 fractional divider to 18*/
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
    /* Configure USDHC clock source and divider */
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
}

/* Put the unused frame buffer to the s_fbList. */
static void APP_PutFrameBuffer(void *fb)
{
    *(void **)fb = s_fbList;
    s_fbList = fb;
}
static int MOUNT_SDCard(void)
{
    // FRESULT error;
    const TCHAR driverName[3U] = {SDDISK + '0', ':', '/'};

    // clear FATFS manually
    memset((void *)&g_fileSystem, 0, sizeof(g_fileSystem));

    /* Wait for the card insert. */
    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        PRINTF("Card not inserted.\r\n");
        return -1;
    }

    // Mount the driver
    if (f_mount(&g_fileSystem, driverName, 0))
    {
        PRINTF("Mount volume failed.\r\n");
        return -2;
    }

#if (FF_FS_RPATH >= 2U)
    if (f_chdrive((char const *)&driverName[0U]))
    {
        PRINTF("Change drive failed.\r\n");
        return -3;
    }
#endif

    // Open file to check
/*
    error = f_open(&jpgFil, _T("/pics/000.jpg"), FA_OPEN_EXISTING);
    if (error != FR_OK)
    {
        PRINTF("No demo jpeg file!\r\n");
        return -4;
    }
    else
    {
        PRINTF("/pics/000.jpg open done!\r\n");
    }
*/
    f_close(&jpgFil);

    return 0;
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


#if 0
void APP_ELCDIF_Init(void)
{
    uint8_t i;

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
        .bufferAddr = (uint32_t)g_frameBuffer[0],
        .pixelFormat = kELCDIF_PixelFormatRGB888,
        .dataBus = APP_LCDIF_DATA_BUS,
    };

    for (i = 1; i < APP_LCD_FB_NUM; i++)
    {
        APP_PutFrameBuffer(g_frameBuffer[i]);
    }

    activeBuf = g_frameBuffer[0];
    ELCDIF_RgbModeInit(APP_ELCDIF, &config);
}

#endif



#define APP_PXP PXP
#define APP_PS_WIDTH 480/* 720,352,image resolution*/
#define APP_BPP 4U

static pxp_output_buffer_config_t outputBufferConfig;
static pxp_ps_buffer_config_t psBufferConfig;
AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_psBufferLcd[2][APP_IMG_HEIGHT][APP_IMG_WIDTH], FRAME_BUFFER_ALIGN);

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

    ELCDIF_RgbModeStart(APP_ELCDIF);
}

//--------------------- decoder ------------------------

AT_NONCACHEABLE_SECTION(static FIL inputFil);
AT_NONCACHEABLE_SECTION(static FIL aoutputFil);
AT_NONCACHEABLE_SECTION(static FIL voutputFil);
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavformat/url.h"
//#include "decoder.h"

extern AVCodec ff_h264_decoder;
extern AVCodec ff_aac_decoder;
//extern AVCodecParser ff_h264_parser;
extern AVInputFormat ff_mov_demuxer;
extern URLProtocol ff_file_protocol;

#define AVCODEC_MAX_AUDIO_FRAME_SIZE 100
#define AVCODEC_MAX_VIDEO_FRAME_SIZE 4096

#define APP_PS_ULC_X 0U
#define APP_PS_ULC_Y 0U

static void LCD_display(unsigned char *buf[], int xsize,int ysize)
{
    uint8_t curLcdBufferIdx = 1U;
    static uint32_t isPxpOn = 0;
    psBufferConfig.bufferAddr = (uint32_t)buf[0];
    psBufferConfig.bufferAddrU = (uint32_t)buf[1];
    psBufferConfig.bufferAddrV = (uint32_t)buf[2];

    PXP_SetProcessSurfaceBufferConfig(APP_PXP, &psBufferConfig);

    PXP_SetProcessSurfaceScaler(APP_PXP, xsize, ysize, APP_IMG_WIDTH, APP_IMG_HEIGHT);
    PXP_SetProcessSurfacePosition(APP_PXP, APP_PS_ULC_X, APP_PS_ULC_Y, APP_PS_ULC_X + APP_IMG_WIDTH - 1U,
                                          APP_PS_ULC_Y + APP_IMG_HEIGHT - 1U);
    outputBufferConfig.buffer0Addr = (uint32_t)s_psBufferLcd[curLcdBufferIdx];
    PXP_SetOutputBufferConfig(APP_PXP, &outputBufferConfig);

	if (isPxpOn) {
	  // while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(APP_PXP))) {}
		PXP_ClearStatusFlags(APP_PXP, kPXP_CompleteFlag);
	}
    PXP_Start(APP_PXP);
	isPxpOn = 1;

   ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)s_psBufferLcd[curLcdBufferIdx]);
   ELCDIF_ClearInterruptStatus(APP_ELCDIF, kELCDIF_CurFrameDone);

   /*
   while (!(kELCDIF_CurFrameDone & ELCDIF_GetInterruptStatus(APP_ELCDIF)))
   {
   }
   */
   curLcdBufferIdx ^= 1U;
}
//static int decode_write_frame(FIL *file, AVCodecContext *avctx, AVFrame *frame, int *frame_index, AVPacket *pkt, int flush)
//{
//        int got_frame = 0;
//	do {
//		int len = avcodec_decode_video2(avctx, frame, &got_frame, pkt);
//		if (len < 0) {
//			//fprintf(stderr, "Error while decoding frame %d\n", *frame_index);
//                        printf("Error while decoding frame %d\n!", *frame_index);
//			return len;
//		}
//
//		if (got_frame) {
//                       printf("%d\r\n", *frame_index);
//
//			if (file) {
//
//                               LCD_display(frame->data, frame->linesize, frame->width, frame->height);
//			}
//                        (*frame_index)++;
//		}
//	} while (flush && got_frame);
//	return 0;
//}

void config_sai(uint32_t bitWidth, uint32_t sampleRate_Hz, sai_mono_stereo_t stereo);
void play_audio(uint8_t *audioData, uint32_t audioBytes);

typedef enum _conv_audio_format
{
    kConvAudioFormat_Int16 = 0U,
    kConvAudioFormat_Int24 = 1U,
    kConvAudioFormat_Int32 = 2U,
} conv_audio_format_t;

void convert_audio_format(float *src,
                          uint8_t *dest,
                          uint32_t samples,
                          conv_audio_format_t format)
{
    for (uint32_t i = 0; i < samples; i++)
    {
        if ((*src) > 1.0f)
        {
            *src = 1.0f;
        }
        else if ((*src) < -1.0f)
        {
            *src = -1.0f;
        }
        if (format == kConvAudioFormat_Int16)
        {
            int32_t temp = (*src) * ((int32_t)1 << 16);
            *((int16_t *)dest) = (int16_t)temp;
            dest +=sizeof(int16_t);
        }
        else if (format == kConvAudioFormat_Int24)
        {
            int32_t temp = (*src) * ((int32_t)1 << 24);
            *((int32_t *)dest) = (int32_t)temp;
            dest +=sizeof(int32_t);
        }
        else if (format == kConvAudioFormat_Int32)
        {
            int64_t temp = (double)(*src) * ((int64_t)1 << 32);
            *((int32_t *)dest) = (int32_t)temp;
            dest +=sizeof(int32_t);
        }
        src++;
    }
}

static void h264_video_decode(const char *infilename, const char *aoutfilename, const char *voutfilename)
{
    printf("Decode file '%s' to '%s' and '%s'\n", infilename, aoutfilename, voutfilename);
    // Check input/output media file
    FRESULT c;
    c = f_open(&inputFil, infilename, FA_READ | FA_OPEN_EXISTING);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", infilename, c);
    }
    c = f_open(&aoutputFil, aoutfilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", aoutfilename, c);
    }
    c = f_open(&voutputFil, voutfilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", voutfilename, c);
    }

    // Define user format conext var
    AVFormatContext *pInFmtCtx=NULL;
    // Register several FFmpeg own var
    av_register_input_format(&ff_mov_demuxer);
    ffurl_register_protocol(&ff_file_protocol);
    avcodec_register(&ff_h264_decoder);
    avcodec_register(&ff_aac_decoder);
    // Init user format conext var
    pInFmtCtx = avformat_alloc_context();
    // Open input media file (mp4) from SD Card
    int error =avformat_open_input(&pInFmtCtx, infilename, NULL, NULL);
    if (error != 0)
    {
        printf("Couldn't open file: error :%d\n",error);
    }
    // Get steam info from input media file
    error = avformat_find_stream_info(pInFmtCtx, NULL);
    if (error < 0)
    {
        printf("Couldn't find stream information error :%d\n",error);
    }
    int audioStream = -1;
    int videoStream = -1;
    for (unsigned int j = 0; j < pInFmtCtx->nb_streams; j++)
    {
        // Find audio steam index
        if(pInFmtCtx->streams[j]->codec->codec_type==AVMEDIA_TYPE_AUDIO)
        {
            audioStream = j;
        }
        // Find video steam index
        if(pInFmtCtx->streams[j]->codec->codec_type==AVMEDIA_TYPE_VIDEO)
        {
            videoStream = j;
        }
    }
    printf("audio stream num: %d\n",audioStream);
    printf("video stream num: %d\n",videoStream);
    //
    AVCodecContext *pInCodecCtx_video = pInFmtCtx->streams[videoStream]->codec;
    AVCodec *pInCodec_video = avcodec_find_decoder(AV_CODEC_ID_H264);
    if(avcodec_open2(pInCodecCtx_video, pInCodec_video, NULL) < 0)
    {
        printf("error avcodec_open failed video.\n");

    }
    AVCodecContext *pInCodecCtx_audio = pInFmtCtx->streams[audioStream]->codec;
    AVCodec *pInCodec_audio = avcodec_find_decoder(AV_CODEC_ID_AAC);
    if(avcodec_open2(pInCodecCtx_audio, pInCodec_audio, NULL) < 0)
    {
        printf("error avcodec_open failed audio.\n");

    }

    AVPacket packet;
    UINT *bw_wh;
    AVFrame *frame = av_frame_alloc();
    AVFrame *frameyuv = av_frame_alloc();

    uint8_t *pingAudioBuffer = (uint8_t *)av_malloc(4096);
    uint8_t *pongAudioBuffer = (uint8_t *)av_malloc(4096);
    bool isPingBufferAvail = true;

    while (av_read_frame(pInFmtCtx, &packet) >= 0)
    {
        if (packet.stream_index == audioStream)
        {
            uint8_t *pktdata = packet.data;
            int pktsize = packet.size;
            int out_size;
            int len;
            while (pktsize > 0)
            {
                out_size = AVCODEC_MAX_AUDIO_FRAME_SIZE * 100;
                len =avcodec_decode_audio4(pInCodecCtx_audio, frame, &out_size, &packet);
                if (len < 0)
                {
                    printf("Error while decoding audio.\n");
                    break;
                }
                if (out_size > 0)
                {
                    if (pInCodecCtx_audio->sample_fmt == AV_SAMPLE_FMT_S16P)
                    { // Audacity: 16bit PCM little endian stereo
                        int16_t* ptr_l = (int16_t*)frame->extended_data[0];
                        int16_t* ptr_r = (int16_t*)frame->extended_data[1];
                        for (int i = 0; i < frame->nb_samples; i++)
                        {
                            f_write(&aoutputFil, ptr_l++, sizeof(int16_t), &bw_wh);
                            f_write(&aoutputFil, ptr_r++, sizeof(int16_t), &bw_wh);
                        }
                    }
                    else if (pInCodecCtx_audio->sample_fmt == AV_SAMPLE_FMT_FLTP)
                    { //Audacity: little endian 32bit stereo start offset 7 ()
                        float* ptr_l = (float*)frame->extended_data[0];
                        float* ptr_r = (float*)frame->extended_data[1];
                        uint8_t *audioBuffer = isPingBufferAvail ? pingAudioBuffer : pongAudioBuffer;
                        isPingBufferAvail = !isPingBufferAvail;
                        convert_audio_format(ptr_r, audioBuffer, frame->nb_samples, kConvAudioFormat_Int24);
                        play_audio(audioBuffer, sizeof(int32_t) * frame->nb_samples);
                        //for (int i = 0; i < frame->nb_samples; i++)
                        //{
                        //    f_write(&aoutputFil, ptr_l++, sizeof(float), &bw_wh);
                        //    f_write(&aoutputFil, ptr_r++, sizeof(float), &bw_wh);
                        //}
                    }
                }
                pktsize -= len;
                pktdata += len;
            }
        }
        if (packet.stream_index == videoStream)
        {
            uint8_t *pktdatayuv = packet.data;
            int pktsizeyuv = packet.size;
            int out_sizeyuv;
            int lenyuv;
            while (pktsizeyuv > 0)
            {
                out_sizeyuv = AVCODEC_MAX_VIDEO_FRAME_SIZE * 100;
                lenyuv =avcodec_decode_video2(pInCodecCtx_video, frameyuv, &out_sizeyuv, &packet);
                if (lenyuv < 0)
                {
                    printf("Error while decoding video.\n");
                    break;
                }
                if (out_sizeyuv > 0)
                {
                    // Show video (yuv444p) via LCD, note: PXP can only support YUV444->RGB24
                    LCD_display(frameyuv->data, frameyuv->width, frameyuv->height);
                    // Save YUV data (yuv420p) in file
                    //int y_size = frameyuv->width * frameyuv->height;
                    //f_write(&voutputFil, frameyuv->data[0], y_size, &bw_wh);
                    //f_write(&voutputFil, frameyuv->data[1], y_size/4, &bw_wh);
                    //f_write(&voutputFil, frameyuv->data[2], y_size/4, &bw_wh);
                }
                pktsizeyuv -= lenyuv;
                pktdatayuv += lenyuv;
            }
        }
        //av_free_packet(&packet);
        av_packet_unref(&packet);
    }

    f_close(&inputFil);
    f_close(&aoutputFil);
    f_close(&voutputFil);
    avcodec_close(pInCodecCtx_video);
    avcodec_close(pInCodecCtx_audio);
    avformat_close_input(&pInFmtCtx);
    avformat_free_context(pInFmtCtx);
    printf("Decode Done\n");
}


/*!
 * @brief Main function
 */
int main(void)
{
    FRESULT error;
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitLcdifPixelClock();
    BOARD_USDHCClockConfiguration();
    BOARD_InitDebugConsole();
    BOARD_InitLcd();

    PRINTF("MP4 decode demo start:\r\n");

    // Init the SD card
    if (0 != MOUNT_SDCard())
    {
        PRINTF("SD card mount error. Demo stopped!");
        return -1;
    }

    config_sai(kSAI_WordWidth24bits, kSAI_SampleRate44100Hz, kSAI_MonoRight);

    // clear the framebuffer first
    memset(g_frameBuffer, 0, sizeof(g_frameBuffer));

    ELCDIF_EnableInterrupts(APP_ELCDIF, kELCDIF_CurFrameDoneInterruptEnable);
    ELCDIF_RgbModeStart(APP_ELCDIF);

    APP_InitPxp();
    APP_InitLcdif();
    BOARD_EnableLcdInterrupt();

    /*decoder*/
    //char *filepath_in="/bigbuckbunny_480x272.h264";
    //char *filepath_in="/clown_720x576.h264";
    //char *filepath_in="/formen_352x288.h264";
    //char *filepath_in="/test.h264";
    //char *filepath_in="/1.h64";
    //char *filepath_out="/test-out.yuv";
    char *filepath_in="/tmall.mp4";
    char *filepath_aout="/tmall.pcm";
    char *filepath_vout="/tmall.yuv";
    //while(1)
    h264_video_decode(filepath_in, filepath_aout, filepath_vout);
}

static status_t sdcardWaitCardInsert(void)
{
    /* Save host information. */
    g_sd.host.base = SD_HOST_BASEADDR;
    g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
    /* card detect type */
    g_sd.usrParam.cd = &s_sdCardDetect;
#if defined DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
    g_sd.usrParam.pwr = &s_sdCardPwrCtrl;
#endif
    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }
    /* power off card */
    SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);
    /* wait card insert */
    if (SD_WaitCardDetectStatus(SD_HOST_BASEADDR, &s_sdCardDetect, true) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power on the card */
        SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}
