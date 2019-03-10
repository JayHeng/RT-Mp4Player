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
#include "ff.h"
#include "fsl_sd.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "fsl_sai.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum _conv_audio_format
{
    kConvAudioFormat_Int16 = 0U,
    kConvAudioFormat_Int24 = 1U,
    kConvAudioFormat_Int32 = 2U,
} conv_audio_format_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t sdcard_wait_insert(void);
static int sdcard_mount(void);

extern void config_sai(uint32_t bitWidth, uint32_t sampleRate_Hz, sai_mono_stereo_t stereo);
extern void sai_audio_play(uint8_t *audioData, uint32_t audioBytes);
extern void config_lcd(void);
extern void lcd_video_display(unsigned char *buf[], int xsize, int ysize);
extern void config_gpt(void);
extern void time_measure_start(void);
extern uint64_t time_measure_done(void);

#if MP4_WAV_ENABLE
void wav_start(uint32_t bitWidth, uint32_t sampleRate_Hz, uint32_t channels, uint32_t fileSize);
void wav_write(uint8_t *data, uint32_t bytes);
void wav_close(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */

extern sd_card_t g_sd; /* sd card descriptor */
volatile bool sdcard = false;

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
    .powerOn = BOARD_PowerOnSDCARD,
    .powerOnDelay_ms = 500U,
    .powerOff = BOARD_PowerOffSDCARD,
    .powerOffDelay_ms = 0U,
};
#endif

typedef struct _time_measure_context
{
    uint64_t readFrame_ns;
    uint64_t decodeAudio_ns;
    uint64_t convAudio_ns;
    uint64_t playAudio_ns;
    uint64_t decodeVideo_ns;
    uint64_t playVideo_ns;
    bool isAudioStream;
} time_measure_context_t;

#define TIME_MEASURE_CNT 20
static time_measure_context_t s_timeMeasureContext[TIME_MEASURE_CNT];
static uint32_t s_timeMeasureIndex = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

//--------------------- SD/eMMC ------------------------
static status_t sdcard_wait_insert(void)
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

static int sdcard_mount(void)
{
    // FRESULT error;
    const TCHAR driverName[3U] = {SDDISK + '0', ':', '/'};

    // clear FATFS manually
    memset((void *)&g_fileSystem, 0, sizeof(g_fileSystem));

    /* Wait for the card insert. */
    if (sdcard_wait_insert() != kStatus_Success)
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

    return 0;
}

//--------------------- FFmpeg ------------------------

AT_NONCACHEABLE_SECTION(static FIL inputFil);
AT_NONCACHEABLE_SECTION(static FIL aoutputFil);
AT_NONCACHEABLE_SECTION(static FIL voutputFil);
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavformat/url.h"

extern AVCodec ff_h264_decoder;
extern AVCodec ff_aac_decoder;
//extern AVCodecParser ff_h264_parser;
extern AVInputFormat ff_mov_demuxer;
extern URLProtocol ff_file_protocol;

#define AVCODEC_MAX_AUDIO_FRAME_SIZE 100
#define AVCODEC_MAX_VIDEO_FRAME_SIZE 4096

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

    s_timeMeasureIndex = 0;
    time_measure_start();

#if MP4_WAV_ENABLE
    wav_start(kSAI_WordWidth16bits, kSAI_SampleRate44100Hz, 1, 3874877);
#endif

    while (av_read_frame(pInFmtCtx, &packet) >= 0)
    {
        s_timeMeasureContext[s_timeMeasureIndex].readFrame_ns = time_measure_done();
        if (packet.stream_index == audioStream)
        {
            s_timeMeasureContext[s_timeMeasureIndex].isAudioStream = true;
            uint8_t *pktdata = packet.data;
            int pktsize = packet.size;
            int out_size;
            int len;
            while (pktsize > 0)
            {
                out_size = AVCODEC_MAX_AUDIO_FRAME_SIZE * 100;
                time_measure_start();
                len =avcodec_decode_audio4(pInCodecCtx_audio, frame, &out_size, &packet);
                s_timeMeasureContext[s_timeMeasureIndex].decodeAudio_ns = time_measure_done();
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
                        convert_audio_format(ptr_r, audioBuffer, frame->nb_samples, kConvAudioFormat_Int16);
                        time_measure_start();
                        sai_audio_play(audioBuffer, sizeof(int16_t) * frame->nb_samples);
                        s_timeMeasureContext[s_timeMeasureIndex].playAudio_ns = time_measure_done();

#if MP4_WAV_ENABLE
                        wav_write(audioBuffer, frame->nb_samples * sizeof(int16_t));
#endif
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
            s_timeMeasureContext[s_timeMeasureIndex].isAudioStream = false;
            uint8_t *pktdatayuv = packet.data;
            int pktsizeyuv = packet.size;
            int out_sizeyuv;
            int lenyuv;
            while (pktsizeyuv > 0)
            {
                out_sizeyuv = AVCODEC_MAX_VIDEO_FRAME_SIZE * 100;
                time_measure_start();
                lenyuv =avcodec_decode_video2(pInCodecCtx_video, frameyuv, &out_sizeyuv, &packet);
                s_timeMeasureContext[s_timeMeasureIndex].decodeVideo_ns = time_measure_done();
                if (lenyuv < 0)
                {
                    printf("Error while decoding video.\n");
                    break;
                }
                if (out_sizeyuv > 0)
                {
                    // Show video (yuv444p) via LCD, note: PXP can only support YUV444->RGB24
                    time_measure_start();
                    lcd_video_display(frameyuv->data, frameyuv->width, frameyuv->height);
                    s_timeMeasureContext[s_timeMeasureIndex].playVideo_ns = time_measure_done();
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
        time_measure_start();
        if (s_timeMeasureIndex < TIME_MEASURE_CNT)
        {
            s_timeMeasureIndex++;
        }
    }

#if MP4_WAV_ENABLE
    wav_close();
#endif

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
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("MP4 decode demo start:\r\n");

    /*configure system pll PFD2 fractional divider to 18*/
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
    /* Configure USDHC clock source and divider */
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);

    // Init the SD card
    if (0 != sdcard_mount())
    {
        PRINTF("SD card mount error. Demo stopped!");
        return -1;
    }

    // Init SAI module
    config_sai(kSAI_WordWidth16bits, kSAI_SampleRate44100Hz, kSAI_MonoRight);
    
    // Init LCD module
    config_lcd();

    // Init GPT module
    config_gpt();

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
