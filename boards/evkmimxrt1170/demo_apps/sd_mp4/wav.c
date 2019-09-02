/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
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
#include "fsl_sd.h"
#include "ff.h"
#include "diskio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void wav_header(uint8_t *header, uint32_t sampleRate, uint32_t bitsPerFrame, uint8_t channels, uint32_t fileSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION(static FIL s_wavOutputFil);

/*******************************************************************************
 * Code
 ******************************************************************************/
void wav_header(uint8_t *header, uint32_t sampleRate, uint32_t bitsPerFrame, uint8_t channels, uint32_t fileSize)
{
    uint32_t totalDataLen = fileSize - 8U;
    uint32_t audioDataLen = fileSize - 44U;
    uint32_t byteRate = sampleRate * (bitsPerFrame / 8U) * channels;
    header[0] = 'R';
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    header[4] = (totalDataLen & 0xff); /* file-size (equals file-size - 8) */
    header[5] = ((totalDataLen >> 8U) & 0xff);
    header[6] = ((totalDataLen >> 16U) & 0xff);
    header[7] = ((totalDataLen >> 24U) & 0xff);
    header[8] = 'W'; /* Mark it as type "WAVE" */
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f'; /* Mark the format section 'fmt ' chunk */
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    header[16] = 16; /* 4 bytes: size of 'fmt ' chunk, Length of format data.  Always 16 */
    header[17] = 0;
    header[18] = 0;
    header[19] = 0;
    header[20] = 1; /* format = 1 ,Wave type PCM */
    header[21] = 0;
    header[22] = channels; /* channels */
    header[23] = 0;
    header[24] = (sampleRate & 0xff);
    header[25] = ((sampleRate >> 8U) & 0xff);
    header[26] = ((sampleRate >> 16U) & 0xff);
    header[27] = ((sampleRate >> 24U) & 0xff);
    header[28] = (byteRate & 0xff);
    header[29] = ((byteRate >> 8U) & 0xff);
    header[30] = ((byteRate >> 16U) & 0xff);
    header[31] = ((byteRate >> 24U) & 0xff);
    header[32] = (channels * bitsPerFrame / 8); /* block align */
    header[33] = 0;
    header[34] = bitsPerFrame; /* bits per sample */
    header[35] = 0;
    header[36] = 'd'; /*"data" marker */
    header[37] = 'a';
    header[38] = 't';
    header[39] = 'a';
    header[40] = (audioDataLen & 0xff); /* data-size (equals file-size - 44).*/
    header[41] = ((audioDataLen >> 8) & 0xff);
    header[42] = ((audioDataLen >> 16) & 0xff);
    header[43] = ((audioDataLen >> 24) & 0xff);
}

void wav_start(uint32_t bitWidth, uint32_t sampleRate_Hz, uint32_t channels, uint32_t fileSize)
{
    uint32_t bytesWritten = 0;
    uint8_t header[44] = {0};
    char *filepath_in="/tmall.wav";
    FRESULT error = f_open(&s_wavOutputFil, filepath_in, FA_CREATE_ALWAYS | FA_WRITE);
    if (error)
    {
        if (error == FR_EXIST)
        {
            printf("File exists.\r\n");
        }
        else
        {
            printf("Open file failed.\r\n");
            return;
        }
    }

    /* Write the data into the sdcard */
    wav_header(header, sampleRate_Hz, bitWidth, channels, fileSize);

    /* Write the wav header */
    error = f_write(&s_wavOutputFil, (void *)header, 44U, (UINT *)&bytesWritten);
    if ((error) || (bytesWritten != 44))
    {
        printf("Write file failed. \r\n");
    }
}

void wav_write(uint8_t *data, uint32_t bytes)
{
    uint32_t bytesWritten = 0;

    f_write(&s_wavOutputFil, data, bytes, (UINT *)&bytesWritten);
}

void wav_close(void)
{
    f_close(&s_wavOutputFil);
}

