/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_asrc_m2m_edma.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.asrc_m2m_edma"
#endif

/*******************************************************************************
 * Definitations
 ******************************************************************************/
/* Used for 32byte aligned */
#define STCD_ADDR(address) (edma_tcd_t *)(((uint32_t)(address) + 32) & ~0x1FU)

/*<! Structure definition for uart_edma_private_handle_t. The structure is private. */
typedef struct _asrc_edma_private_handle
{
    ASRC_Type *base;
    asrc_edma_handle_t *handle;
} asrc_edma_private_handle_t;

/*<! Private handle only used for internally. */
static asrc_edma_private_handle_t s_edmaPrivateHandle[FSL_FEATURE_SOC_ASRC_COUNT][FSL_ASRC_CHANNEL_PAIR_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief ASRC EDMA callback for input.
 *
 * @param handle pointer to asrc_edma_handle_t structure which stores the transfer state.
 * @param userData Parameter for user callback.
 * @param done If the DMA transfer finished.
 * @param tcds The TCD index.
 */
static void ASRC_InEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds);

/*!
 * @brief ASRC EDMA callback for output.
 *
 * @param handle pointer to asrc_edma_handle_t structure which stores the transfer state.
 * @param userData Parameter for user callback.
 * @param done If the DMA transfer finished.
 * @param tcds The TCD index.
 */
static void ASRC_OutEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds);
/*******************************************************************************
 * Code
 ******************************************************************************/
static void ASRC_InEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    asrc_edma_private_handle_t *privHandle = (asrc_edma_private_handle_t *)userData;
    asrc_edma_handle_t *asrcHandle         = privHandle->handle;
    asrc_in_edma_handle_t *asrcInHandle    = &(privHandle->handle->in);

    /* If finished a block, call the callback function */
    asrcInHandle->asrcQueue[asrcInHandle->queueDriver] = NULL;
    asrcInHandle->queueDriver                          = (asrcInHandle->queueDriver + 1) % ASRC_XFER_QUEUE_SIZE;
    if (asrcHandle->callback)
    {
        (asrcHandle->callback)(privHandle->base, asrcHandle, kStatus_ASRCInIdle, asrcHandle->userData);
    }

    /* If all data finished, just stop the transfer */
    if (asrcInHandle->asrcQueue[asrcInHandle->queueDriver] == NULL)
    {
        EDMA_AbortTransfer(asrcInHandle->inDmaHandle);
    }
}

static void ASRC_ReadFIFORemainedSampleEDMA(
    ASRC_Type *base, asrc_channel_pair_t channelPair, uint32_t *outAddr, uint32_t outWidth, uint32_t size)
{
    uint32_t *addr = outAddr;
    uint32_t i     = 0U;

    for (i = 0U; i < size / outWidth; i++)
    {
        ASRC_GetRemainFifoSamples(base, channelPair, addr, outWidth, 1U);
        addr = (uint32_t *)((uint32_t)addr + outWidth);
    }
}

static void ASRC_OutEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    asrc_edma_private_handle_t *privHandle = (asrc_edma_private_handle_t *)userData;
    asrc_edma_handle_t *asrcHandle         = privHandle->handle;
    asrc_out_edma_handle_t *asrcOutHandle  = &(privHandle->handle->out);
    uint32_t queueDriverIndex              = asrcOutHandle->queueDriver;
    status_t callbackStatus                = kStatus_ASRCOutIdle;

    /* If finished a block, call the callback function */
    asrcOutHandle->queueDriver = (queueDriverIndex + 1) % ASRC_XFER_QUEUE_SIZE;

    /* If all data finished, just stop the transfer */
    if (asrcOutHandle->asrcQueue[asrcOutHandle->queueDriver] == NULL)
    {
        if (asrcOutHandle->pollingSize)
        {
            uint32_t *outPollingAddr = (uint32_t *)((uint32_t)asrcHandle->out.asrcQueue[queueDriverIndex] +
                                                    asrcHandle->out.transferSize[queueDriverIndex]);
            ASRC_ReadFIFORemainedSampleEDMA(privHandle->base, asrcHandle->channelPair, outPollingAddr,
                                            asrcOutHandle->sampleWidth, asrcOutHandle->pollingSize);
        }
        EDMA_AbortTransfer(asrcOutHandle->outDmaHandle);
        callbackStatus = kStatus_ASRCOutQueueIdle;
    }

    if (asrcHandle->callback)
    {
        (asrcHandle->callback)(privHandle->base, asrcHandle, callbackStatus, asrcHandle->userData);
    }

    asrcOutHandle->asrcQueue[queueDriverIndex] = NULL;
}

/*!
 * brief Initializes the ASRC m2m eDMA handle.
 *
 * This function initializes the ASRC DMA handle, which can be used for other ASRC transactional APIs.
 * Usually, for a specified ASRC channel pair, call this API once to get the initialized handle.
 *
 * param base ASRC base pointer.
 * param channelPair ASRC channel pair
 * param handle ASRC eDMA handle pointer.
 * param callback Pointer to user callback function.
 * param txDmaHandle ASRC send edma handle pointer.
 * param rxDmaHandle ASRC read edma handle pointer
 * param userData User parameter passed to the callback function.
 */
void ASRC_TransferCreateHandleEDMA(ASRC_Type *base,
                                   asrc_edma_handle_t *handle,
                                   asrc_channel_pair_t channelPair,
                                   asrc_edma_callback_t callback,
                                   edma_handle_t *inDmaHandle,
                                   edma_handle_t *outDmaHandle,
                                   void *userData)
{
    assert(handle && inDmaHandle && outDmaHandle);

    uint32_t instance = ASRC_GetInstance(base);

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    handle->in.inDmaHandle   = inDmaHandle;
    handle->out.outDmaHandle = outDmaHandle;
    handle->callback         = callback;
    handle->userData         = userData;

    /* Set ASRC state to idle */
    handle->state       = kStatus_ASRCIdle;
    handle->channelPair = channelPair;

    s_edmaPrivateHandle[instance][channelPair].base   = base;
    s_edmaPrivateHandle[instance][channelPair].handle = handle;

    /* Need to use scatter gather */
    EDMA_InstallTCDMemory(inDmaHandle, (edma_tcd_t *)(STCD_ADDR(handle->in.tcd)), ASRC_XFER_QUEUE_SIZE);
    EDMA_InstallTCDMemory(outDmaHandle, (edma_tcd_t *)(STCD_ADDR(handle->out.tcd)), ASRC_XFER_QUEUE_SIZE);

    /* Install callback for Tx dma channel */
    EDMA_SetCallback(inDmaHandle, ASRC_InEDMACallback, &s_edmaPrivateHandle[instance][channelPair]);
    EDMA_SetCallback(outDmaHandle, ASRC_OutEDMACallback, &s_edmaPrivateHandle[instance][channelPair]);
}

/*!
 * brief Configures the ASRC m2m channel pair.
 *
 *
 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 * param asrcConfig asrc configurations.
 * param inputSampleRate ASRC input sample rate.
 * param outputSampleRate ASRC output sample rate.
 */
status_t ASRC_TransferSetChannelPairConfigEDMA(ASRC_Type *base,
                                               asrc_edma_handle_t *handle,
                                               asrc_channel_pair_config_t *asrcConfig,
                                               uint32_t inSampleRate,
                                               uint32_t outSampleRate)
{
    assert(handle && asrcConfig);

    /* Configure the audio format to ASRC registers */
    if (ASRC_SetChannelPairConfig(base, handle->channelPair, asrcConfig, inSampleRate, outSampleRate))
    {
        return kStatus_ASRCChannelPairConfigureFailed;
    }

    handle->in.fifoThreshold  = asrcConfig->inFifoThreshold * asrcConfig->audioDataChannels;
    handle->out.fifoThreshold = asrcConfig->outFifoThreshold * asrcConfig->audioDataChannels;
    ASRC_MapSamplesWidth(base, handle->channelPair, &handle->in.sampleWidth, &handle->out.sampleWidth);

    return kStatus_Success;
}

/*!
 * brief Performs a non-blocking ASRC m2m convert using EDMA.
 *
 * note This interface returns immediately after the transfer initiates.

 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 * param xfer Pointer to the DMA transfer structure.
 * retval kStatus_Success Start a ASRC eDMA send successfully.
 * retval kStatus_InvalidArgument The input argument is invalid.
 * retval kStatus_ASRCQueueFull ASRC EDMA driver queue is full.
 */
status_t ASRC_TransferEDMA(ASRC_Type *base, asrc_edma_handle_t *handle, asrc_transfer_t *xfer)
{
    assert(handle && xfer);

    edma_transfer_config_t config = {0};
    uint32_t inAddr               = ASRC_GetInputDataRegisterAddress(base, handle->channelPair);
    uint32_t outAddr              = ASRC_GetOutputDataRegisterAddress(base, handle->channelPair);
    uint32_t nonAlignSize         = xfer->outDataSize % (handle->out.fifoThreshold * handle->out.sampleWidth);

    /* Check if input parameter invalid */
    if ((xfer->inData == NULL) || (xfer->inDataSize == 0U) || (xfer->outData == NULL) || (xfer->outDataSize == 0U))
    {
        return kStatus_InvalidArgument;
    }

    if ((handle->in.asrcQueue[handle->in.queueUser]) || (handle->out.asrcQueue[handle->out.queueUser]))
    {
        return kStatus_ASRCQueueFull;
    }

    /* Add into queue */
    handle->in.asrcQueue[handle->in.queueUser]    = xfer->inData;
    handle->in.transferSize[handle->in.queueUser] = xfer->inDataSize;
    handle->in.queueUser                          = (handle->in.queueUser + 1) % ASRC_XFER_QUEUE_SIZE;

    handle->out.pollingSize += nonAlignSize;
    handle->out.asrcQueue[handle->out.queueUser]    = xfer->outData;
    handle->out.transferSize[handle->out.queueUser] = xfer->outDataSize - nonAlignSize;
    handle->out.queueUser                           = (handle->out.queueUser + 1) % ASRC_XFER_QUEUE_SIZE;

    /* Prepare ASRC input edma configuration */
    EDMA_PrepareTransfer(&config, (void *)outAddr, handle->out.sampleWidth, xfer->outData, handle->out.sampleWidth,
                         handle->out.fifoThreshold * handle->out.sampleWidth, xfer->outDataSize - nonAlignSize,
                         kEDMA_PeripheralToMemory);
    EDMA_SubmitTransfer(handle->out.outDmaHandle, &config);
    EDMA_StartTransfer(handle->out.outDmaHandle);

    /* Prepare ASRC output edma configuration */
    EDMA_PrepareTransfer(&config, xfer->inData, handle->in.sampleWidth, (void *)inAddr, handle->in.sampleWidth,
                         handle->in.fifoThreshold * handle->in.sampleWidth, xfer->inDataSize, kEDMA_MemoryToPeripheral);
    EDMA_SubmitTransfer(handle->in.inDmaHandle, &config);
    EDMA_StartTransfer(handle->in.inDmaHandle);

    /* Change the state of handle */
    handle->state = kStatus_ASRCBusy;

    return kStatus_Success;
}

/*!
 * brief Aborts a ASRC m2m transfer using eDMA.
 *
 * This function only aborts the current transfer slots, the other transfer slots' information still kept
 * in the handler. If users want to terminate all transfer slots, just call ASRC_TransferTerminateConvertEDMA.
 *
 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferAbortEDMA(ASRC_Type *base, asrc_edma_handle_t *handle)
{
    assert(handle);

    /* Disable dma */
    EDMA_AbortTransfer(handle->in.inDmaHandle);
    EDMA_AbortTransfer(handle->out.outDmaHandle);

    /* Handle the queue index */
    handle->in.asrcQueue[handle->in.queueDriver] = NULL;
    handle->in.queueDriver                       = (handle->in.queueDriver + 1) % ASRC_XFER_QUEUE_SIZE;

    /* Set the handle state */
    handle->state = kStatus_ASRCIdle;
}

/*!
 * brief Terminate all ASRC Convert.
 *
 * This function will clear all transfer slots buffered in the asrc queue. If users only want to abort the
 * current transfer slot, please call ASRC_TransferAbortConvertEDMA.
 *
 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferAbortTerminalEDMA(ASRC_Type *base, asrc_edma_handle_t *handle)
{
    assert(handle);

    /* Abort the current transfer */
    ASRC_TransferAbortEDMA(base, handle);

    /* Clear all the internal information */
    memset(handle->in.tcd, 0U, sizeof(handle->in.tcd));
    memset(handle->in.asrcQueue, 0U, sizeof(handle->in.asrcQueue));
    memset(handle->in.transferSize, 0U, sizeof(handle->in.transferSize));
    handle->in.queueUser   = 0U;
    handle->in.queueDriver = 0U;

    memset(handle->out.tcd, 0U, sizeof(handle->out.tcd));
    memset(handle->out.asrcQueue, 0U, sizeof(handle->out.asrcQueue));
    memset(handle->out.transferSize, 0U, sizeof(handle->out.transferSize));
    handle->out.queueUser   = 0U;
    handle->out.queueDriver = 0U;
}

/*!
 * brief Gets byte count converted by ASRC.
 *
 * param base ASRC base pointer
 * param handle ASRC eDMA handle pointer.
 * param count Bytes count received by ASRC.
 * retval kStatus_Success Succeed get the transfer count.
 * retval kStatus_ASRCIdle There is no non-blocking transaction in progress.
 */
status_t ASRC_TransferGetConvertedSamplesEDMA(ASRC_Type *base, asrc_edma_handle_t *handle, size_t *count)
{
    assert(handle);

    status_t status = kStatus_Success;

    if (handle->state != kStatus_ASRCBusy)
    {
        status = kStatus_ASRCIdle;
    }
    else
    {
        *count = (handle->in.transferSize[handle->in.queueDriver] -
                  (uint32_t)handle->in.fifoThreshold * handle->in.sampleWidth *
                      EDMA_GetRemainingMajorLoopCount(handle->in.inDmaHandle->base, handle->in.inDmaHandle->channel));
    }

    return status;
}
