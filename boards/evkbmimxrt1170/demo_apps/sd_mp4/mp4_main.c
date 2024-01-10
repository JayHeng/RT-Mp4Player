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
#include "sdmmc_config.h"
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
extern void lcd_video_display(uint8_t *buf[], uint32_t xsize, uint32_t ysize);
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

/*******************************************************************************
 * Code
 ******************************************************************************/

//--------------------- SD/eMMC ------------------------
static status_t sdcard_wait_insert(void)
{
    BOARD_SD_Config(&g_sd, NULL, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }
    /* power off card */
    SD_SetCardPower(&g_sd, false);
    /* wait card insert */
    if (SD_PollingCardInsert(&g_sd, kSD_Inserted) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power on the card */
        SD_SetCardPower(&g_sd, true);
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
    
    AVCodecContext *pInCodecCtx_audio;
    AVCodec *pInCodec_audio;
    if (audioStream != -1)
    {
        pInCodecCtx_audio = pInFmtCtx->streams[audioStream]->codec;
        pInCodec_audio = avcodec_find_decoder(AV_CODEC_ID_AAC);
        if(avcodec_open2(pInCodecCtx_audio, pInCodec_audio, NULL) < 0)
        {
            printf("error avcodec_open failed audio.\n");
            return;
        }
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
    GPIO_PinWrite(BOARD_USER_LED_GPIO, BOARD_USER_LED_GPIO_PIN, 1U);

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

    char *filepath_in="/bigBuckBunny_480x272_faststart.mp4";
    char *filepath_aout="/bigBuckBunny_480x272_faststart.pcm";
    char *filepath_vout="/bigBuckBunny_480x272_faststart.yuv";

    while(1)
    {
        h264_video_decode(filepath_in, filepath_aout, filepath_vout);
    }
}
