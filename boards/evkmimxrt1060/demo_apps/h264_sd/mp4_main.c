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
#include "fsl_gpio.h"
#include "ff.h"
#include "fsl_sd.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "fsl_sai.h"
#include "mp4.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t sdcard_wait_insert(void);
static int sdcard_mount(void);

extern audio_sai_cfg_t g_audioSaiCfg;
extern void config_sai(audio_sai_cfg_t *saiCfg);
extern void sai_audio_play(uint8_t *audioData, uint32_t audioBytes);
extern video_lcd_cfg_t g_videoLcdCfg;
extern void config_lcd(video_lcd_cfg_t *lcdCfg);
extern void lcd_video_display(uint32_t activeBufferAddr);
extern camera_csi_cfg_t g_cameraCsiCfg;
extern uint32_t config_csi(camera_csi_cfg_t *csiCfg);
extern void csi_camera_capture(uint32_t *activeBufferAddr, uint32_t *inactiveBufferAddr);

extern void config_gpt(void);

static void flush_audio_data_cache(void);

#if MP4_WAV_ENABLE == 1
static void wav_start(uint32_t bitWidth, uint32_t sampleRate_Hz, uint32_t channels, uint32_t fileSize);
static void wav_write(uint8_t *data, uint32_t bytes);
static void wav_close(void);
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

// Audio format: 2 channel(Max) , int32_t(Max)
static uint8_t s_audioBufferQueue[AUDIO_BUFFER_QUEUE][4*2*AUDIO_FRAME_SIZE*AUDIO_CACHE_FRAMES] @ ".audioBufferQueue";
static uint8_t s_audioBufferQueueIndex = 0;

static uint8_t *s_cachedAudioBuffer;
static uint32_t s_cachedAudioBytes = 0;
static uint32_t s_cachedAudioFrames = 0;

extern uint8_t g_psBufferLcd[APP_LCD_FB_NUM][APP_IMG_HEIGHT][APP_IMG_WIDTH][APP_BPP];

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
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavutil/imgutils.h"
#include "libavformat/avformat.h"
#include "libavformat/url.h"

extern AVCodec ff_h264_decoder;
extern AVCodec ff_aac_decoder;
//extern AVCodecParser ff_h264_parser;
extern AVInputFormat ff_mov_demuxer;
extern URLProtocol ff_file_protocol;

#define AVCODEC_MAX_AUDIO_FRAME_SIZE 100
#define AVCODEC_MAX_VIDEO_FRAME_SIZE 4096

void convert_audio_format(float *src_l,
                          float *src_r,
                          uint8_t *dest,
                          uint32_t samples,
                          uint32_t format_intBits)
{
    bool hasCh_l = src_l != 0;
    bool hasCh_r = src_r != 0;
    if ((!hasCh_l) && (!hasCh_r))
    {
        return;
    }
    for (uint32_t i = 0; i < samples; i++)
    {
        float data_l, data_r;
        if (hasCh_l)
        {
            if ((*src_l) > 1.0f)
            {
                data_l = 1.0f;
            }
            else if ((*src_l) < -1.0f)
            {
                data_l = -1.0f;
            }
            else
            {
                data_l = *src_l;
            }
            if (format_intBits == 16)
            {
                int32_t temp = (int32_t)(data_l * ((int32_t)1 << 16));
                *((int16_t *)dest) = (int16_t)temp;
                dest +=sizeof(int16_t);
            }
            else if (format_intBits == 24)
            {
                int32_t temp = (int32_t)(data_l * ((int32_t)1 << 24));
                *((int32_t *)dest) = (int32_t)temp;
                dest +=sizeof(int32_t);
            }
            else if (format_intBits == 32)
            {
                int64_t temp = (int64_t)((double)data_l * ((int64_t)1 << 32));
                *((int32_t *)dest) = (int32_t)temp;
                dest +=sizeof(int32_t);

            }
        }
        if (hasCh_r)
        {
            if ((*src_r) > 1.0f)
            {
                data_r = 1.0f;
            }
            else if ((*src_r) < -1.0f)
            {
                data_r = -1.0f;
            }
            else
            {
                data_r = *src_r;
            }
            if (format_intBits == 16)
            {
                int32_t temp = (int32_t)(data_r * ((int32_t)1 << 16));
                *((int16_t *)dest) = (int16_t)temp;
                dest +=sizeof(int16_t);
            }
            else if (format_intBits == 24)
            {
                int32_t temp = (int32_t)(data_r * ((int32_t)1 << 24));
                *((int32_t *)dest) = (int32_t)temp;
                dest +=sizeof(int32_t);
            }
            else if (format_intBits == 32)
            {
                int64_t temp = (int64_t)((double)data_r * ((int64_t)1 << 32));
                *((int32_t *)dest) = (int32_t)temp;
                dest +=sizeof(int32_t);
            }
        }
        src_l++;
        src_r++;
    }
}

static void flush_audio_data_cache(void)
{
    if (s_cachedAudioBytes)
    {
        sai_audio_play(s_cachedAudioBuffer, s_cachedAudioBytes);
        s_cachedAudioFrames = 0;
        s_cachedAudioBytes = 0;
    }
}

#if MP4_FF_TIME_ENABLE == 1
extern void time_measure_start(void);
extern uint64_t time_measure_done(void);
#define FF_MEASURE_FRAMES 2000
AT_NONCACHEABLE_SECTION(static FIL toutputFil);
static ff_measure_context_t s_ffMeasureContext;
static uint32_t s_ffMeasureIndex = 0;
static uint8_t s_hexStrBuffer[97] = "0x0000000000000000,0x0000000000000000,0x0000000000000000,0x0000000000000000,0x0000000000000000,\r\n";
static void byte_to_hex_str(uint8_t *hexBuf, uint64_t data)
{
    for (uint32_t i = 0; i < 16; i++)
    {
        uint8_t loc = (15 - i) * 4;
        uint8_t hex = (data & ((uint64_t)0xf << loc)) >> loc;
        if (hex < 0xa)
        {
            hexBuf[i] = hex + '0';
        }
        else
        {
            hexBuf[i] = hex - 0xa + 'a';
        }
    }
}
#if MP4_LCD_TIME_ENABLE == 1
extern lcd_measure_context_t g_lcdMeasureContext;
#endif
#endif // #if MP4_FF_TIME_ENABLE

typedef enum _ff_time_type
{
    kFfTimeType_Start       = 0U,
    kFfTimeType_ReadFrame   = 1U,
    kFfTimeType_DecodeAudio = 2U,
    kFfTimeType_DecodeVideo = 3U,
} ff_time_type_t;

static void ff_time_measure_utility(ff_time_type_t type)
{
#if MP4_FF_TIME_ENABLE == 1
    if (type == kFfTimeType_Start)
    {
        time_measure_start();
    }
    else if (type == kFfTimeType_ReadFrame)
    {
        s_ffMeasureContext.readFrame_ns = time_measure_done();
    }
    else if (type == kFfTimeType_DecodeAudio)
    {
        s_ffMeasureContext.isAudioStream = true;
        s_ffMeasureContext.decodeAudio_ns = time_measure_done();
    }
    else if (type == kFfTimeType_DecodeVideo)
    {
        s_ffMeasureContext.isAudioStream = false;
        s_ffMeasureContext.decodeVideo_ns = time_measure_done();
    }
#else
    // Do nothing
#endif
}

/*
static void h264_video_decode(const char *infilename, const char *aoutfilename, const char *voutfilename)
{
    printf("Decode file '%s' to '%s' and '%s'\n", infilename, aoutfilename, voutfilename);
    // Check input/output media file
    FRESULT c;
    c = f_open(&inputFil, infilename, FA_READ | FA_OPEN_EXISTING);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", infilename, c);
        return;
    }
    c = f_open(&aoutputFil, aoutfilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", aoutfilename, c);
        return;
    }
    c = f_open(&voutputFil, voutfilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", voutfilename, c);
        return;
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
        return;
    }
    // Get steam info from input media file
    error = avformat_find_stream_info(pInFmtCtx, NULL);
    if (error < 0)
    {
        printf("Couldn't find stream information error :%d\n",error);
        return;
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
        return;

    }
    AVCodecContext *pInCodecCtx_audio = pInFmtCtx->streams[audioStream]->codec;
    AVCodec *pInCodec_audio = avcodec_find_decoder(AV_CODEC_ID_AAC);
    if(avcodec_open2(pInCodecCtx_audio, pInCodec_audio, NULL) < 0)
    {
        printf("error avcodec_open failed audio.\n");
        return;
    }

    AVPacket packet;
    UINT *bw_wh;
    AVFrame *frame = av_frame_alloc();
    AVFrame *frameyuv = av_frame_alloc();

#if MP4_FF_TIME_ENABLE == 1
    char *toutfilename="/time.txt";
    c = f_open(&toutputFil, toutfilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (c != FR_OK)
    {
        return;
    }
    s_ffMeasureIndex = 0;
#endif
    ff_time_measure_utility(kFfTimeType_Start);

#if MP4_WAV_ENABLE == 1
    wav_start(kSAI_WordWidth16bits, kSAI_SampleRate44100Hz, 1, 3874877);
#endif

    while (av_read_frame(pInFmtCtx, &packet) >= 0)
    {
        ff_time_measure_utility(kFfTimeType_ReadFrame);
        if (packet.stream_index == audioStream)
        {
            uint8_t *pktdata = packet.data;
            int pktsize = packet.size;
            int out_size;
            int len;
            while (pktsize > 0)
            {
                out_size = AVCODEC_MAX_AUDIO_FRAME_SIZE * 100;
                ff_time_measure_utility(kFfTimeType_Start);
                len =avcodec_decode_audio4(pInCodecCtx_audio, frame, &out_size, &packet);
                ff_time_measure_utility(kFfTimeType_DecodeAudio);
                if (len < 0)
                {
                    printf("Error while decoding audio.\n");
                    break;
                }
                if (out_size > 0)
                {
                    if (!g_audioSaiCfg.isSaiConfigured)
                    {
                        g_audioSaiCfg.sampleRate_Hz = frame->sample_rate;
                        if (frame->channels >= 2)
                        {
                            g_audioSaiCfg.sampleChannel = 2;
                        }
                        else
                        {
                            g_audioSaiCfg.sampleChannel = frame->channels;
                        }
                        config_sai(&g_audioSaiCfg);
                        g_audioSaiCfg.isSaiConfigured = true;
                    }

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
                        if (g_audioSaiCfg.sampleChannel < 2)
                        {
                            ptr_r = 0;
                        }
                        uint8_t *audioBuffer = s_audioBufferQueue[s_audioBufferQueueIndex];
                        convert_audio_format(ptr_l, ptr_r, audioBuffer + s_cachedAudioBytes, frame->nb_samples, g_audioSaiCfg.sampleWidth_bit);
                        s_cachedAudioBuffer = audioBuffer;
                        s_cachedAudioBytes += (g_audioSaiCfg.sampleWidth_bit / 8) * frame->nb_samples * g_audioSaiCfg.sampleChannel;
                        s_cachedAudioFrames++;
                        if (s_cachedAudioFrames == AUDIO_CACHE_FRAMES)
                        {
                            flush_audio_data_cache();
                            s_audioBufferQueueIndex++;
                            s_audioBufferQueueIndex %= AUDIO_BUFFER_QUEUE;
                        }
#if MP4_WAV_ENABLE == 1
                        wav_write(audioBuffer, (g_audioSaiCfg.sampleWidth_bit / 8) * frame->nb_samples * g_audioSaiCfg.sampleChannel);
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
        else if (packet.stream_index == videoStream)
        {
            uint8_t *pktdatayuv = packet.data;
            int pktsizeyuv = packet.size;
            int out_sizeyuv;
            int lenyuv;
            while (pktsizeyuv > 0)
            {
                out_sizeyuv = AVCODEC_MAX_VIDEO_FRAME_SIZE * 100;
                ff_time_measure_utility(kFfTimeType_Start);
                lenyuv =avcodec_decode_video2(pInCodecCtx_video, frameyuv, &out_sizeyuv, &packet);
                ff_time_measure_utility(kFfTimeType_DecodeVideo);
                if (lenyuv < 0)
                {
                    printf("Error while decoding video.\n");
                    break;
                }
                if (out_sizeyuv > 0)
                {
                    if (!g_videoLcdCfg.isLcdConfigured)
                    {
                        g_videoLcdCfg.srcWidth = frameyuv->width;
                        g_videoLcdCfg.srcHeight = frameyuv->height;
                        config_lcd(&g_videoLcdCfg);
                        g_videoLcdCfg.isLcdConfigured = true;
                    }
                    // Show video (yuv444p) via LCD, note: PXP can only support YUV444->RGB24
                    lcd_video_display(frameyuv->data, frameyuv->width, frameyuv->height);
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

#if MP4_FF_TIME_ENABLE == 1
        byte_to_hex_str(&s_hexStrBuffer[2], s_ffMeasureContext.readFrame_ns);
        if (s_ffMeasureContext.isAudioStream)
        {
            byte_to_hex_str(&s_hexStrBuffer[21], s_ffMeasureContext.decodeAudio_ns);
            byte_to_hex_str(&s_hexStrBuffer[40], 0);
            byte_to_hex_str(&s_hexStrBuffer[59], 0);
            byte_to_hex_str(&s_hexStrBuffer[78], 0);
        }
        else
        {
            byte_to_hex_str(&s_hexStrBuffer[21], 0);
            byte_to_hex_str(&s_hexStrBuffer[40], s_ffMeasureContext.decodeVideo_ns);
#if MP4_LCD_TIME_ENABLE == 1
            byte_to_hex_str(&s_hexStrBuffer[59], g_lcdMeasureContext.costTimePxp_ns);
            byte_to_hex_str(&s_hexStrBuffer[78], g_lcdMeasureContext.costTimeLcd_ns);
#endif
        }
        f_write(&toutputFil, s_hexStrBuffer, sizeof(s_hexStrBuffer), &bw_wh);
#if FF_MEASURE_FRAMES
        if (s_ffMeasureIndex < FF_MEASURE_FRAMES)
        {
            s_ffMeasureIndex++;
        }
        else
        {
            break;
        }
#endif
#endif
        ff_time_measure_utility(kFfTimeType_Start);
    }

#if MP4_FF_TIME_ENABLE == 1
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    GPIO_PinInit(BOARD_USER_LED_GPIO, BOARD_USER_LED_GPIO_PIN, &led_config);
    GPIO_PinWrite(BOARD_USER_LED_GPIO, BOARD_USER_LED_GPIO_PIN, 0U);

    f_close(&toutputFil);
#endif

#if MP4_WAV_ENABLE == 1
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
*/

int flush_encoder(AVFormatContext *fmt_ctx, unsigned int stream_index)
{
    int ret;
    int got_frame;
    AVPacket enc_pkt;
    if (!(fmt_ctx->streams[stream_index]->codec->codec->capabilities & CODEC_CAP_DELAY))
    {
        return 0;
    }
    while (1)
    {
        enc_pkt.data = NULL;
        enc_pkt.size = 0;
        av_init_packet(&enc_pkt);
        ret = avcodec_encode_video2 (fmt_ctx->streams[stream_index]->codec, &enc_pkt, NULL, &got_frame);
        av_frame_free(NULL);
        if (ret < 0)
        {
            break;
        }
        if (!got_frame)
        {
            ret=0;
            break;
        }
        printf("Flush Encoder: Succeed to encode 1 frame!\tsize:%5d\n",enc_pkt.size);
        /* mux encoded frame */
        ret = av_write_frame(fmt_ctx, &enc_pkt);
        if (ret < 0)
        {
            break;
        }
    }
    return ret;
}

static void h264_file_encode(const char *vinfilename, const char *voutfilename)
{
    printf("Encode video from '%s' to '%s'\n", vinfilename, voutfilename);

    FRESULT c;
    c = f_open(&inputFil, vinfilename, FA_READ | FA_OPEN_EXISTING);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", vinfilename, c);
        return;
    }
    c = f_open(&voutputFil, voutfilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (c != FR_OK)
    {
        printf("Could not open '%s', erro = %d\n", voutfilename, c);
        return;
    }

    AVFormatContext* pFormatCtx;
    AVOutputFormat* fmt;
    AVStream* video_st;
    AVCodecContext* pCodecCtx;
    AVCodec* pCodec;
    AVPacket pkt;
    uint8_t* picture_buf;
    AVFrame* pFrame;
    int picture_size;
    int y_size;
    int framecnt = 0;
    int in_w=480, in_h=272;                              //Input data's width and height
    int framenum = 100;                                   //Frames to encode

    ffurl_register_protocol(&ff_file_protocol);
    pFormatCtx = avformat_alloc_context();
    //Guess Format
    fmt = av_guess_format(NULL, voutfilename, NULL);
    pFormatCtx->oformat = fmt;

    //Open output URL
    if (avio_open(&pFormatCtx->pb, voutfilename, AVIO_FLAG_READ_WRITE) < 0)
    {
        printf("Failed to open output file! \n");
        return;
    }

    pCodec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!pCodec)
    {
        printf("Can not find encoder! \n");
        return;
    }

    video_st = avformat_new_stream(pFormatCtx, pCodec);
    if (video_st == NULL)
    {
        return;
    }

    //Param that must set
    pCodecCtx = video_st->codec;
    pCodecCtx->codec_id = fmt->video_codec;
    pCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
    pCodecCtx->width = in_w;
    pCodecCtx->height = in_h;
    pCodecCtx->bit_rate = 400000;
    pCodecCtx->gop_size=250;

    pCodecCtx->time_base.num = 1;
    pCodecCtx->time_base.den = 25;

    //H264
    pCodecCtx->qmin = 10;
    pCodecCtx->qmax = 51;

    //Optional Param
    pCodecCtx->max_b_frames=3;

    // Set Option
    AVDictionary *param = 0;
    //H.264
    if(pCodecCtx->codec_id == AV_CODEC_ID_H264)
    {
        av_dict_set(&param, "preset", "slow", 0);
        av_dict_set(&param, "tune", "zerolatency", 0);
    }

    //Show some Information
    av_dump_format(pFormatCtx, 0, voutfilename, 1);

    if (avcodec_open2(pCodecCtx, pCodec, &param) < 0)
    {
        printf("Failed to open encoder! \n");
        return;
    }

    pFrame = av_frame_alloc();
    picture_size = avpicture_get_size(pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height);
    picture_buf = (uint8_t *)av_malloc(picture_size);
    avpicture_fill((AVPicture *)pFrame, picture_buf, pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height);

    //Write File Header
    avformat_write_header(pFormatCtx, NULL);

    av_new_packet(&pkt,picture_size);

    y_size = pCodecCtx->width * pCodecCtx->height;

    for (int i = 0; i < framenum; i++)
    {
        //Read raw YUV data
        uint32_t* bytesRead;
        if (f_read(&inputFil, picture_buf, y_size*3/2, bytesRead) <= 0)
        {
            printf("Failed to read raw data! \n");
            return;
        }
        else if(f_eof(&inputFil))
        {
            break;
        }
        pFrame->data[0] = picture_buf;              // Y
        pFrame->data[1] = picture_buf+ y_size;      // U
        pFrame->data[2] = picture_buf+ y_size*5/4;  // V
        //PTS
        pFrame->pts = i*(video_st->time_base.den)/((video_st->time_base.num)*25);
        int got_picture = 0;
        //Encode
        int ret = avcodec_encode_video2(pCodecCtx, &pkt, pFrame, &got_picture);
        if(ret < 0)
        {
            printf("Failed to encode! \n");
            return;
        }
        if (got_picture == 1)
        {
            printf("Succeed to encode frame: %5d\t size:%5d\n", framecnt, pkt.size);
            framecnt++;
            pkt.stream_index = video_st->index;
            ret = av_write_frame(pFormatCtx, &pkt);
            av_free_packet(&pkt);
        }
    }

    //Flush Encoder
    int ret = flush_encoder(pFormatCtx, 0);
    if (ret < 0)
    {
        printf("Flushing encoder failed\n");
        return;
    }

    //Write file trailer
    av_write_trailer(pFormatCtx);

    //Clean
    if (video_st)
    {
        avcodec_close(video_st->codec);
        av_free(pFrame);
        av_free(picture_buf);
    }
    avio_close(pFormatCtx->pb);
    avformat_free_context(pFormatCtx);

    f_close(&inputFil);
}

static void h264_file_encode_pure(const char *vinfilename, const char *voutfilename)
{
    AVCodec *pCodec;
    AVCodecContext *pCodecCtx= NULL;
    int i, ret, got_output;
    FILE *fp_in;
	FILE *fp_out;
    AVFrame *pFrame;
    AVPacket pkt;
	int y_size;
	int framecnt=0;

	char filename_in[]="../ds_480x272.yuv";

#if TEST_HEVC
	AVCodecID codec_id=AV_CODEC_ID_HEVC;
	char filename_out[]="ds.hevc";
#else
	AVCodecID codec_id=AV_CODEC_ID_H264;
	char filename_out[]="ds.h264";
#endif


	int in_w=480,in_h=272;	
	int framenum=100;	

	avcodec_register_all();

    pCodec = avcodec_find_encoder(codec_id);
    if (!pCodec) {
        printf("Codec not found\n");
        return -1;
    }
    pCodecCtx = avcodec_alloc_context3(pCodec);
    if (!pCodecCtx) {
        printf("Could not allocate video codec context\n");
        return -1;
    }
    pCodecCtx->bit_rate = 400000;
    pCodecCtx->width = in_w;
    pCodecCtx->height = in_h;
    pCodecCtx->time_base.num=1;
	pCodecCtx->time_base.den=25;
    pCodecCtx->gop_size = 10;
    pCodecCtx->max_b_frames = 1;
    pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;

    if (codec_id == AV_CODEC_ID_H264)
        av_opt_set(pCodecCtx->priv_data, "preset", "slow", 0);
 
    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
        printf("Could not open codec\n");
        return -1;
    }
    
    pFrame = av_frame_alloc();
    if (!pFrame) {
        printf("Could not allocate video frame\n");
        return -1;
    }
    pFrame->format = pCodecCtx->pix_fmt;
    pFrame->width  = pCodecCtx->width;
    pFrame->height = pCodecCtx->height;

    ret = av_image_alloc(pFrame->data, pFrame->linesize, pCodecCtx->width, pCodecCtx->height,
                         pCodecCtx->pix_fmt, 16);
    if (ret < 0) {
        printf("Could not allocate raw picture buffer\n");
        return -1;
    }
	//Input raw data
	fp_in = fopen(filename_in, "rb");
	if (!fp_in) {
		printf("Could not open %s\n", filename_in);
		return -1;
	}
	//Output bitstream
	fp_out = fopen(filename_out, "wb");
	if (!fp_out) {
		printf("Could not open %s\n", filename_out);
		return -1;
	}

	y_size = pCodecCtx->width * pCodecCtx->height;
    //Encode
    for (i = 0; i < framenum; i++) {
        av_init_packet(&pkt);
        pkt.data = NULL;    // packet data will be allocated by the encoder
        pkt.size = 0;
		//Read raw YUV data
		if (fread(pFrame->data[0],1,y_size,fp_in)<= 0||		// Y
			fread(pFrame->data[1],1,y_size/4,fp_in)<= 0||	// U
			fread(pFrame->data[2],1,y_size/4,fp_in)<= 0){	// V
			return -1;
		}else if(feof(fp_in)){
			break;
		}

        pFrame->pts = i;
        /* encode the image */
        ret = avcodec_encode_video2(pCodecCtx, &pkt, pFrame, &got_output);
        if (ret < 0) {
            printf("Error encoding frame\n");
            return -1;
        }
        if (got_output) {
            printf("Succeed to encode frame: %5d\tsize:%5d\n",framecnt,pkt.size);
			framecnt++;
            fwrite(pkt.data, 1, pkt.size, fp_out);
            av_free_packet(&pkt);
        }
    }
    //Flush Encoder
    for (got_output = 1; got_output; i++) {
        ret = avcodec_encode_video2(pCodecCtx, &pkt, NULL, &got_output);
        if (ret < 0) {
            printf("Error encoding frame\n");
            return -1;
        }
        if (got_output) {
            printf("Flush Encoder: Succeed to encode 1 frame!\tsize:%5d\n",pkt.size);
            fwrite(pkt.data, 1, pkt.size, fp_out);
            av_free_packet(&pkt);
        }
    }

    fclose(fp_out);
    avcodec_close(pCodecCtx);
    av_free(pCodecCtx);
    av_freep(&pFrame->data[0]);
    av_frame_free(&pFrame);

	return 0;
}

static void h264_camera_encode(const char *voutfilename)
{
    printf("Encode video from camera to '%s'\n", voutfilename);

    uint32_t frameAddrToDisplay;
    uint32_t frameAddrToCapture;

    // Config CSI and get one empty frame buffer
    frameAddrToCapture = config_csi(&g_cameraCsiCfg);
    g_cameraCsiCfg.isCsiConfigured = true;

    // Get one full frame buffer according to the empty frame buffer address
    for (uint32_t i = 0; i < APP_LCD_FB_NUM; i++)
    {
       if (frameAddrToCapture != (uint32_t)(g_psBufferLcd[i]))
       {
            frameAddrToDisplay = (uint32_t)(g_psBufferLcd[i]);
            break;
       }
    }

    // Config LCD
    config_lcd(&g_videoLcdCfg);
    g_videoLcdCfg.isLcdConfigured = true;

    while (1)
    {
        lcd_video_display(frameAddrToDisplay);
        csi_camera_capture(&frameAddrToCapture, &frameAddrToDisplay);
        frameAddrToDisplay = frameAddrToCapture;
    }
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

    PRINTF("H264 encode demo start:\r\n");

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

#if (MP4_SAI_TIME_ENABLE == 1) || (MP4_LCD_TIME_ENABLE == 1) || (MP4_FF_TIME_ENABLE == 1)
    // Init GPT module
    config_gpt();
#endif

    g_audioSaiCfg.isSaiConfigured = false;
    g_audioSaiCfg.sampleWidth_bit = AUDIO_CONV_WIDTH;
    g_videoLcdCfg.isLcdConfigured = false;

    char *filepath_vin="/SampleVideo_480x272_10s_yuv420p.yuv";
    char *filepath_vout="/SampleVideo_480x272_10s_yuv420p.h264";

    while(1)
    {
        h264_file_encode_pure(filepath_vin, filepath_vout);
    }
}
