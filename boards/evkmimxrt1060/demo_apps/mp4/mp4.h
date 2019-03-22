/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

// Set to save decoded audio data into .wav file
#define MP4_WAV_ENABLE      0
// Set to measure FFmpeg decode time for each frame
#define MP4_FF_TIME_ENABLE  0
// Set to measure SAI transfer time for cached frames
#define MP4_SAI_TIME_ENABLE 0
// The detected error time in ns for one frame
#define AUDIO_FRAME_ERR_NS 1000000

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
#define VIDEO_PXP_BLOCKING 1

// Set video resolution
#define VIDEO_RESOLUTION_272P   0  // For 480*272 LCD
#define VIDEO_RESOLUTION_720HD  1  // For 1280*800 LCD

// Set video pixel format
#define VIDEO_PIXEL_FMT_RGB888  0
#define VIDEO_PIXEL_FMT_RGB565  1

// Set video refresh frequency
#define VIDEO_REFRESH_FREG_60Hz  0
#define VIDEO_REFRESH_FREG_30Hz  1
