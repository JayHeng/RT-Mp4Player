/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

// Set to save decoded audio data into .wav file
#define MP4_WAV_ENABLE      0

typedef struct _ff_measure_context
{
    uint64_t readFrame_ns;
    uint64_t decodeAudio_ns;
    uint64_t decodeVideo_ns;
    bool isAudioStream;
} ff_measure_context_t;

// Set to measure FFmpeg decode time for each frame
#define MP4_FF_TIME_ENABLE  0

typedef struct _sai_measure_context
{
    uint32_t transIndex;
    uint32_t costBytes;
    uint64_t costTimeSai_ns;
} sai_measure_context_t;

// Set to measure SAI transfer time for cached frames
// !!!Note: it can NOT be enabled along with MP4_FF_TIME_ENABLE
#define MP4_SAI_TIME_ENABLE 0
// The detected error time in ns for one frame
#define AUDIO_FRAME_ERR_NS  1000000

typedef struct _lcd_measure_context
{
    uint64_t costTimePxp_ns;
    uint64_t costTimeLcd_ns;
} lcd_measure_context_t;

// Set to measure LCD display time for each frame
// !!!Note: it can be enabled along with MP4_FF_TIME_ENABLE
#define MP4_LCD_TIME_ENABLE 0

////////////////////////////////////////////////////////////////////////////////

// Set audio frame size according to source media
#define AUDIO_FRAME_SIZE   0x400
// Set audio cache frames, it is important for FFmpeg decode
#define AUDIO_CACHE_FRAMES 3
// Set audio buffer queue, it is important for SAI DMA transfer
#define AUDIO_BUFFER_QUEUE 3

typedef enum _conv_audio_format
{
    kConvAudioFormat_Int16 = 0U,
    kConvAudioFormat_Int24 = 1U,
    kConvAudioFormat_Int32 = 2U,
} conv_audio_format_t;

// Set converted audio data format
#define AUDIO_CONV_FORMAT  kConvAudioFormat_Int16
#define AUDIO_CONV_SIZE    int16_t
#define AUDIO_CONV_CHANNEL 2

// Set SAI configurations for audio
#define AUDIO_SAMP_WIDTH   kSAI_WordWidth16bits
#define AUDIO_SAMP_RATE    kSAI_SampleRate48KHz
#define AUDIO_SAMP_CHANNEL kSAI_Stereo

////////////////////////////////////////////////////////////////////////////////

// Set PXP conversation method for video
#define VIDEO_PXP_CONV_BLOCKING       1
#define VIDEO_PXP_CONV_WAITING        1
// Set LCD display method for video
#define VIDEO_LCD_DISP_BLOCKING       1
#define VIDEO_LCD_DISP_WAITING        0

// Set video resolution
#define VIDEO_SRC_RESOLUTION_TGA120   0  // For 192*120 video
#define VIDEO_SRC_RESOLUTION_MGA180   0  // For 288*180 video
#define VIDEO_SRC_RESOLUTION_CGA240   1  // For 320*240 video
#define VIDEO_SRC_RESOLUTION_HVGA272  0  // For 480*272 video
#define VIDEO_SRC_RESOLUTION_SVGA600  0  // For 800*600 video
#define VIDEO_SRC_RESOLUTION_WXGA800  0  // For 1280*800 video

// Set LCD resolution
#define VIDEO_LCD_RESOLUTION_HVGA272  0  // For 480*272 LCD
#define VIDEO_LCD_RESOLUTION_SVGA600  1  // For 800*600 LCD
#define VIDEO_LCD_RESOLUTION_WXGA800  0  // For 1280*800 LCD

// Set PXP converted pixel format
#define VIDEO_PIXEL_FMT_RGB888        0
#define VIDEO_PIXEL_FMT_RGB565        1

// Set LCD refresh frequency
#define VIDEO_LCD_REFRESH_FREG_60Hz   0
#define VIDEO_LCD_REFRESH_FREG_30Hz   0
#define VIDEO_LCD_REFRESH_FREG_25Hz   1
