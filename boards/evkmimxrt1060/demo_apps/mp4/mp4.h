/*
 * Copyright 2017 NXP
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

#define MP4_WAV_ENABLE      0
#define MP4_FF_TIME_ENABLE  0
#define MP4_SAI_TIME_ENABLE 1

#define AUDIO_FRAME_SIZE   0x400
#define AUDIO_CACHE_FRAMES 3
#define AUDIO_FRAME_ERR_NS 1000000

typedef enum _conv_audio_format
{
    kConvAudioFormat_Int16 = 0U,
    kConvAudioFormat_Int24 = 1U,
    kConvAudioFormat_Int32 = 2U,
} conv_audio_format_t;

#define AUDIO_CONV_FORMAT  kConvAudioFormat_Int16
#define AUDIO_CONV_SIZE    int16_t
#define AUDIO_CONV_CHANNEL 2

#define AUDIO_SAMP_WIDTH   kSAI_WordWidth16bits
#define AUDIO_SAMP_RATE    kSAI_SampleRate44100Hz
#define AUDIO_SAMP_CHANNEL kSAI_Stereo