/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mipi_dsi_aux.h"
#include "mcmgr.h"

#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Address of memory, from which the secondary core will boot */
#define CORE1_BOOT_ADDRESS 0x20200000

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern uint32_t Image$$CORE1_REGION$$Base;
extern uint32_t Image$$CORE1_REGION$$Length;
#define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
#elif defined(__ICCARM__)
extern unsigned char core1_image_start[];
#define CORE1_IMAGE_START core1_image_start
#elif (defined(__GNUC__)) && (!defined(__MCUXPRESSO))
extern const char core1_image_start[];
extern const char *core1_image_end;
extern int core1_image_size;
#define CORE1_IMAGE_START ((void *)core1_image_start)
#define CORE1_IMAGE_SIZE  ((void *)core1_image_size)
#endif
typedef void (*aux_callback_t)(uint16_t data, void *param);
/*******************************************************************************
 * Variables
 ******************************************************************************/
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section("rpmsg_sh_mem_section"), used))
#elif defined(__ICCARM__)
#pragma location = "rpmsg_sh_mem_section"
#endif
dsi_aux_write_mem_transfer_t g_xfer;

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif
static volatile uint16_t g_remote_core_ready = 0;
static volatile uint16_t g_remote_send_done = 0;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Callback when Auxiliary core finishes data transfer.
 *
 * @param param Callback parameter passed to AUX.
 */
static void DSI_AUX_Callback(uint16_t data, void *param);

/*******************************************************************************
 * Code
 ******************************************************************************/
#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void)
{
    uint32_t image_size;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__core1_image"
    image_size = (uint32_t)__section_end("__core1_image") - (uint32_t)&core1_image_start;
#elif defined(__GNUC__) && (!defined(__MCUXPRESSO))
    image_size = (uint32_t)core1_image_size;
#endif
    return image_size;
}
#endif

static void AUX_InstallImage(aux_callback_t callback, dsi_aux_handle_t *handle)
{
   /* Initialize MCMGR, install generic event handlers */
    (void)MCMGR_Init();  

#ifdef CORE1_IMAGE_COPY_TO_RAM
    /* This section ensures the secondary core image is copied from flash location to the target RAM memory.
       It consists of several steps: image size calculation and image copying.
       These steps are not required on MCUXpresso IDE which copies the secondary core image to the target memory during
       startup automatically. */
    uint32_t core1_image_size;
    core1_image_size = get_core1_image_size();

    /* Copy Secondary core application from FLASH to the target memory. */
    (void)memcpy((void *)(char *)CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
#endif
    /* Register the application event before starting the secondary core */
    (void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, callback, ((void *)handle));

    /* Boot Secondary core application */
    (void)MCMGR_StartCore(kMCMGR_Core1, (void *)(char *)CORE1_BOOT_ADDRESS, (uint32_t)&g_xfer, kMCMGR_Start_Synchronous);

    /* Wait until the secondary core is ready for communication */
    while (kDSI_AUX_RemoteCoreReady != g_remote_core_ready);
    
     /* First reset the flag of remote status */
    g_remote_send_done = kDSI_AUX_IMageDataSentdDone;
}
/*!
 * brief Create the MIPI DSI handle.
 *
 * This function initializes the MIPI DSI handle which can be used for other transactional APIs.
 *
 * param base MIPI DSI host peripheral base address.
 * param handle Handle pointer.
 * param callback Callback function.
 * param userData User data.
 */
status_t DSI_TransferCreateHandleAUX(const MIPI_DSI_Type *base,
                                          dsi_aux_handle_t *handle,
                                          dsi_aux_callback_t callback,
                                          void *userData)
{
    assert(NULL != handle);

    /* Zero the handle */
    (void)memset(handle, 0, sizeof(*handle));

    /* Initialize the handle */
    handle->dsi      = base;
    handle->callback = callback;
    handle->userData = userData;
    handle->isBusy   = false;

    AUX_InstallImage(DSI_AUX_Callback, handle);

    return kStatus_Success;
}

/*!
 * brief Abort current APB data transfer.
 *
 * param base MIPI DSI host peripheral base address.
 * param handle pointer to dsi_aux_handle_t structure which stores the transfer state.
 */
void DSI_TransferAbortAUX(const MIPI_DSI_Type *base, dsi_aux_handle_t *handle)
{
    assert(NULL != handle);

    if (handle->isBusy)
    {
//        AUX_Reset();
        /* Reset the state to idle. */
        handle->isBusy = false;
    }
}

/*!
 * brief Write display controller video memory using Auxiliary core.
 *
 * Perform data transfer using Auxiliary core, when transfer finished,
 * upper layer could be informed through callback function.
 *
 * param base MIPI DSI host peripheral base address.
 * param handle pointer to dsi_aux_handle_t structure which stores the transfer state.
 * param xfer Pointer to the transfer structure.
 *
 * retval kStatus_Success Data transfer started successfully.
 * retval kStatus_DSI_Busy Failed to start transfer because DSI is busy with pervious transfer.
 * retval kStatus_DSI_NotSupported Transfer format not supported.
 */
status_t DSI_TransferWriteMemoryAUX(const MIPI_DSI_Type *base,
                                         dsi_aux_handle_t *handle,
                                         dsi_aux_write_mem_transfer_t *xfer)
{
    assert(NULL != handle);

    status_t status;

    if (handle->isBusy)
    {
        status = kStatus_DSI_Busy;
    }
    else
    {
        if (((xfer->inputFormat == kDSI_AUX_InputPixelFormatRGB565) &&
             (xfer->outputFormat == kDSI_AUX_OutputPixelFormatRGB565)) ||
            ((xfer->inputFormat == kDSI_AUX_InputPixelFormatRGB888) &&
             (xfer->outputFormat == kDSI_AUX_OutputPixelFormatRGB888)) ||
            ((xfer->inputFormat == kDSI_AUX_InputPixelFormatXRGB8888) &&
             (xfer->outputFormat == kDSI_AUX_OutputPixelFormatRGB888)))
        {
            // DSI_EnableInterrupts(base, (uint32_t)kDSI_InterruptGroup1ApbTxDone | (uint32_t)kDSI_InterruptGroup1HtxTo,
                                 // 0U);
            while (kDSI_AUX_IMageDataSentdDone != g_remote_send_done)
                ;
            g_xfer = *xfer;

            MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, (uint16_t) kDSI_AUX_ImageDataReadyForSent);
            // g_remote_send_done = 0;
            status = kStatus_Success;
        }
        else
        {
            status = kStatus_DSI_NotSupported;
        }
    }

    return status;
}

/*!
 * brief Callback when AUX done.
 *
 * param param Callback parameter passed to AUX.
 */
static void DSI_AUX_Callback(uint16_t data, void *param)
{
    dsi_aux_handle_t *handle = (dsi_aux_handle_t *)param;

    uint32_t intFlags1, intFlags2;
    
    if (kDSI_AUX_RemoteCoreReady == data)
    {
        g_remote_core_ready = data;
    }
    else if (kDSI_AUX_IMageDataSentdDone == data)
    {
        DSI_DisableInterrupts(handle->dsi, (uint32_t)kDSI_InterruptGroup1ApbTxDone | (uint32_t)kDSI_InterruptGroup1HtxTo,
                              0U);

        DSI_GetAndClearInterruptStatus(handle->dsi, &intFlags1, &intFlags2);

        handle->isBusy = false;

        if (NULL != handle->callback)
        {
            handle->callback(handle->dsi, handle, kStatus_Success, handle->userData);
        }
        
        g_remote_send_done = data;
    }
}
