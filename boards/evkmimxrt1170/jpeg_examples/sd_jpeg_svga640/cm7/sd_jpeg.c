/*
 * Copyright 2017-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "fsl_common.h"
#include "app.h"
#include "fsl_debug_console.h"
#include "fsl_cache.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "jpeglib.h"
#include "display_support.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_FB_HEIGHT DEMO_PANEL_HEIGHT
#define APP_FB_WIDTH DEMO_PANEL_WIDTH

/*
 * For better performance, by default, three frame buffers are used in this demo.
 * If the memory region is not large enough, user can define to 2 to override
 * this value.
 */
#ifndef APP_FB_NUM
#define APP_FB_NUM 3 /* LCD frame buffer number. */
#endif

/* Use RGB888 format by default, for some display controllers don't support
 * RGB888, use XRGB8888
 */
#ifndef APP_USE_XRGB8888
#define APP_USE_XRGB8888 0
#endif

#if APP_USE_XRGB8888
#define APP_FB_BPP 4 /* LCD frame buffer byte per pixel, XRGB888 format, 32-bit. */
#define APP_FB_FORMAT kVIDEO_PixelFormatXRGB8888
#else
#define APP_FB_BPP 3 /* LCD frame buffer byte per pixel, RGB888 format, 24-bit. */
#define APP_FB_FORMAT kVIDEO_PixelFormatRGB888
#endif

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

/*
 * For better performance, the frame buffers are located in cachable region and
 * the cache line maintanance is handled in this demo. The start address of
 * frame buffer, and the size of frame buffer, are aligned to the cache line
 * size.
 */
#ifndef APP_FB_ALIGN
#if (APP_CACHE_LINE_SIZE > 0)
#define APP_FB_ALIGN APP_CACHE_LINE_SIZE
#else
#define APP_FB_ALIGN 1
#endif
#endif /* APP_FB_ALIGN */

#ifndef APP_FB_STRIDE_BYTE
#define APP_FB_STRIDE_BYTE (APP_FB_WIDTH * APP_FB_BPP)
#endif

#define APP_FB_SIZE_BYTE (SDK_SIZEALIGN(APP_FB_STRIDE_BYTE * APP_FB_HEIGHT, APP_FB_ALIGN))

#ifndef APP_FB_USE_FIXED_ADDRESS
#define APP_FB_USE_FIXED_ADDRESS 0
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t sdcardWaitCardInsert(void);
static void APP_BufferSwitchOffCallback(void *param, void *switchOffBuffer);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static dc_fb_info_t fbInfo;
/*
 * When new frame buffer sent to display, it might not be shown immediately.
 * Application could use callback to get new frame shown notification, at the
 * same time, when this flag is set, application could write to the older
 * frame buffer.
 */
static volatile bool s_newFrameShown = false;
static void *volatile s_fbList       = NULL; /* List to the frame buffers. */

/* This struct contains the JPEG decompression parameters */
static struct jpeg_decompress_struct cinfo;
/* This struct represents a JPEG error handler */
static struct jpeg_error_mgr jerr;

#if APP_FB_USE_FIXED_ADDRESS
static void *g_frameBuffer[APP_FB_NUM] = {
    (void *)APP_FB0_ADDR,
#if (APP_FB_NUM > 1)
    (void *)APP_FB1_ADDR,
#endif
#if (APP_FB_NUM > 2)
    (void *)APP_FB2_ADDR,
#endif
};
#else
SDK_ALIGN(static uint8_t g_frameBuffer[APP_FB_NUM][APP_FB_SIZE_BYTE], APP_FB_ALIGN);
#endif

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
    .powerOn          = BOARD_PowerOnSDCARD,
    .powerOnDelay_ms  = 500U,
    .powerOff         = BOARD_PowerOffSDCARD,
    .powerOffDelay_ms = 0U,
};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Get the empty frame buffer from the s_fbList. */
static void *APP_GetFrameBuffer(void)
{
    void *fb;

    fb = s_fbList;

    if (NULL != fb)
    {
        s_fbList = *(void **)fb;
    }

    return fb;
}

/* Put the unused frame buffer to the s_fbList. */
static void APP_PutFrameBuffer(void *fb)
{
    *(void **)fb = s_fbList;
    s_fbList     = fb;
}

/*!
 * @brief  Jpeg Decode
 * @param  file:    pointer to the jpg file
 * @param  buff:     pointer to the image line
 */
void jpeg_decode(FIL *file, uint8_t *buffer)
{
    uint8_t *jpg_buffer;
    uint8_t *jpg_buffer_aligned;
    uint8_t *read_pos;
    UINT jpg_size;
    UINT bytesRead;
    UINT bytesRemain;

    // Decode JPEG Image
    JSAMPROW row_pointer[1] = {0}; /* Output row buffer */
    uint32_t row_stride     = 0;   /* physical row width in image buffer */

    // Step 1: allocate and initialize JPEG decompression object
    cinfo.err = jpeg_std_error(&jerr);

    // Step 2: Initialize the JPEG decompression object
    jpeg_create_decompress(&cinfo);

    jpg_size = f_size(file);

    jpg_buffer = (unsigned char *)malloc(jpg_size + 2 * APP_CACHE_LINE_SIZE);
    if (jpg_buffer == NULL)
    {
        PRINTF("Error: memory allocation error\r\n");
        assert(false);
    }
#if APP_CACHE_LINE_SIZE
    jpg_buffer_aligned = (void *)(SDK_SIZEALIGN((uint32_t)jpg_buffer, APP_CACHE_LINE_SIZE));
#else
    jpg_buffer_aligned = jpg_buffer;
#endif

    bytesRemain = jpg_size;
    read_pos    = jpg_buffer_aligned;

    DCACHE_CleanInvalidateByRange((uint32_t)jpg_buffer_aligned, jpg_size);

    while (bytesRemain > 0)
    {
        f_read(file, read_pos, bytesRemain, &bytesRead);
        bytesRemain -= bytesRead;
        read_pos += bytesRead;
    }

    jpeg_mem_src(&cinfo, jpg_buffer_aligned, jpg_size);

    // Step 3: read image parameters with jpeg_read_header()
    jpeg_read_header(&cinfo, true);

    // Step 4: set parameters for decompression
    cinfo.dct_method = JDCT_FLOAT;
    /*
     * Resize to fit the screen, the actual resize rate is:
     * cinfo.scale_num / 8, the cinfo.scale_num must be in the range of 1 ~ 16
     */
    if ((cinfo.image_width * APP_FB_HEIGHT) > (cinfo.image_height * APP_FB_WIDTH))
    {
        cinfo.scale_num = APP_FB_WIDTH * 8 / cinfo.image_width;
    }
    else
    {
        cinfo.scale_num = APP_FB_HEIGHT * 8 / cinfo.image_height;
    }

    if (cinfo.scale_num < 1)
    {
        cinfo.scale_num = 1;
    }
    else if (cinfo.scale_num > 16)
    {
        cinfo.scale_num = 16;
    }

    // Step 5: start decompressor
    jpeg_start_decompress(&cinfo);

    row_stride = APP_FB_STRIDE_BYTE;

    /* Place the output image to the center of the screen. */
    buffer += row_stride * ((APP_FB_HEIGHT - cinfo.output_height) / 2);
    buffer += APP_FB_BPP * ((APP_FB_WIDTH - cinfo.output_width) / 2);

    while (cinfo.output_scanline < cinfo.output_height)
    {
        row_pointer[0] = &buffer[cinfo.output_scanline * row_stride];
        jpeg_read_scanlines(&cinfo, row_pointer, 1);
    }

    // Step 6: Finish decompression
    jpeg_finish_decompress(&cinfo);

    // Step 7: Release JPEG decompression object
    jpeg_destroy_decompress(&cinfo);

    free(jpg_buffer);
}

static int MOUNT_SDCard(void)
{
    FRESULT error;
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
    error = f_open(&jpgFil, _T("/001.jpg"), FA_OPEN_EXISTING);
    if (error != FR_OK)
    {
        PRINTF("No demo jpeg file!\r\n");
        return -4;
    }

    f_close(&jpgFil);

    return 0;
}

void APP_InitDisplay(void)
{
    uint8_t i;
    void *freeFb;

    status_t status;

    for (i = 0; i < APP_FB_NUM; i++)
    {
        memset(g_frameBuffer[i], 0, APP_FB_SIZE_BYTE);
        APP_PutFrameBuffer(g_frameBuffer[i]);
    }

    BOARD_PrepareDisplayController();

    status = g_dc.ops->init(&g_dc);
    if (kStatus_Success != status)
    {
        PRINTF("Display initialization failed\r\n");
        assert(false);
    }

    g_dc.ops->getLayerDefaultConfig(&g_dc, 0, &fbInfo);
    fbInfo.pixelFormat = APP_FB_FORMAT;
    fbInfo.width       = APP_FB_WIDTH;
    fbInfo.height      = APP_FB_HEIGHT;
    fbInfo.strideBytes = APP_FB_STRIDE_BYTE;
    if (kStatus_Success != g_dc.ops->setLayerConfig(&g_dc, 0, &fbInfo))
    {
        PRINTF("Error: Could not configure the display controller\r\n");
        assert(false);
    }

    g_dc.ops->setCallback(&g_dc, 0, APP_BufferSwitchOffCallback, NULL);

    s_newFrameShown = false;

    /* Get the first frame buffer and send to display controller. */
    freeFb = APP_GetFrameBuffer();

    if (freeFb == NULL)
    {
        PRINTF("Error: Could not get frame buffer\r\n");
        assert(false);
    }

    memset(freeFb, 0, APP_FB_SIZE_BYTE);
    g_dc.ops->setFrameBuffer(&g_dc, 0, freeFb);

    /* For the DBI interface display, application must wait for the first
     * frame buffer sent to the panel.
     */
    if ((g_dc.ops->getProperty(&g_dc) & kDC_FB_ReserveFrameBuffer) == 0)
    {
        while (s_newFrameShown == false)
        {
        }
    }

    s_newFrameShown = true;

    g_dc.ops->enableLayer(&g_dc, 0);
}

extern void config_gpt(void);
extern void time_measure_start(void);
extern uint64_t time_measure_done(void);

typedef struct _jpeg_measure_context
{
    uint64_t decode_ns;
    uint64_t show_ns;
} jpeg_measure_context_t;

AT_NONCACHEABLE_SECTION(static FIL toutputFil);
static jpeg_measure_context_t s_jpegMeasureContext;
static uint8_t s_hexStrBuffer[40] = "0x0000000000000000,0x0000000000000000,\r\n";
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

typedef enum _jpeg_time_type
{
    kJpegTimeType_Start  = 0U,
    kJpegTimeType_Decode = 1U,
    kJpegTimeType_Show   = 2U,
} jpeg_time_type_t;

static void jpeg_time_measure_utility(jpeg_time_type_t type)
{
    if (type == kJpegTimeType_Start)
    {
        time_measure_start();
    }
    else if (type == kJpegTimeType_Decode)
    {
        s_jpegMeasureContext.decode_ns = time_measure_done();
    }
    else if (type == kJpegTimeType_Show)
    {
        s_jpegMeasureContext.show_ns = time_measure_done();
    }
}

static uint32_t g_RootClockSemc = 0;
static uint32_t g_RootClockBus = 0;

/*!
 * @brief Main function
 */
int main(void)
{
    int i = 1;
    FRESULT error;
    char jpgFileName[20];
    void *freeFb;
    uint32_t oldIntStat;

    BOARD_InitHardware();

    PRINTF("SD JPEG demo start:\r\n");

    APP_InitDisplay();

    PRINTF("\n\n\rM7 clock = %d MHz", SystemCoreClock);
    g_RootClockSemc = CLOCK_GetRootClockFreq(kCLOCK_Root_Semc);
    PRINTF("\n\n\rSEMC clock = %d MHz", g_RootClockSemc);
    g_RootClockBus = CLOCK_GetRootClockFreq(kCLOCK_Root_Bus);
    PRINTF("\n\n\rBUS clock = %d MHz", g_RootClockBus);

    // Init the SD card
    if (0 != MOUNT_SDCard())
    {
        PRINTF("SD card mount error. Demo stopped!");
        return -1;
    }

    UINT bw_wh;
    char *toutfilename="/time1.txt";
    FRESULT c = f_open(&toutputFil, toutfilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (c != FR_OK)
    {
        return;
    }
    config_gpt();

    while (1)
    {
        // format the filename
        sprintf(jpgFileName, "/%03d.jpg", i++);
        // open it
        error = f_open(&jpgFil, jpgFileName, FA_READ);
        if (error != FR_OK)
        {
            i = 1;
            f_close(&toutputFil);
            //continue;
            while (1);
        }

        /* Get free frame buffer and convert the jpeg output to it. */
        do
        {
            oldIntStat = DisableGlobalIRQ();
            freeFb     = APP_GetFrameBuffer();
            EnableGlobalIRQ(oldIntStat);
        } while (NULL == freeFb);

        //PRINTF("Decoding %s...", jpgFileName);
        jpeg_time_measure_utility(kJpegTimeType_Start);
        jpeg_decode(&jpgFil, freeFb);
        jpeg_time_measure_utility(kJpegTimeType_Decode);
        //PRINTF("done!\r\n", jpgFileName);
        f_close(&jpgFil);

        DCACHE_CleanInvalidateByRange((uint32_t)freeFb, APP_FB_SIZE_BYTE);

        /*
         * Wait for the previous set frame buffer active.
         */
        jpeg_time_measure_utility(kJpegTimeType_Start);
        while (s_newFrameShown == false)
        {
        }
        jpeg_time_measure_utility(kJpegTimeType_Show);

        /* Now new frame is ready, pass it to LCDIF. */
        s_newFrameShown = false;
        g_dc.ops->setFrameBuffer(&g_dc, 0, freeFb);

        byte_to_hex_str(&s_hexStrBuffer[2], s_jpegMeasureContext.decode_ns);
        byte_to_hex_str(&s_hexStrBuffer[21], s_jpegMeasureContext.show_ns);
        f_write(&toutputFil, s_hexStrBuffer, sizeof(s_hexStrBuffer), &bw_wh);
    }
}

static status_t sdcardWaitCardInsert(void)
{
    /* Save host information. */
    g_sd.host.base           = SD_HOST_BASEADDR;
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

static void APP_BufferSwitchOffCallback(void *param, void *switchOffBuffer)
{
    s_newFrameShown = true;
    APP_PutFrameBuffer(switchOffBuffer);
}
