/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_asrc_p2p_edma.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.asrc_p2p_edma"
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
    asrc_p2p_edma_handle_t *handle;
} asrc_edma_private_handle_t;

/*<! Private handle only used for internally. */
static asrc_edma_private_handle_t s_edmaPrivateHandle[FSL_FEATURE_SOC_ASRC_COUNT][FSL_ASRC_CHANNEL_PAIR_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief ASRC EDMA callback for input.
 *
 * @param handle pointer to asrc_p2p_edma_handle_t structure which stores the transfer state.
 * @param userData Parameter for user callback.
 * @param done If the DMA transfer finished.
 * @param tcds The TCD index.
 */
static void ASRC_InEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds);
/*******************************************************************************
 * Code
 ******************************************************************************/
static void ASRC_InEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    asrc_edma_private_handle_t *privHandle  = (asrc_edma_private_handle_t *)userData;
    asrc_p2p_edma_handle_t *asrcHandle      = privHandle->handle;
    asrc_p2p_in_edma_handle_t *asrcInHandle = &(privHandle->handle->in);
    status_t inStatus                       = kStatus_ASRCInIdle;
    /* If finished a block, call the callback function */
    asrcInHandle->asrcQueue[asrcInHandle->queueDriver] = NULL;
    asrcInHandle->queueDriver                          = (asrcInHandle->queueDriver + 1) % ASRC_XFER_QUEUE_SIZE;

    /* If all data finished, just stop the transfer */
    if (asrcInHandle->asrcQueue[asrcInHandle->queueDriver] == NULL)
    {
        EDMA_AbortTransfer(asrcInHandle->inDmaHandle);
        inStatus = kStatus_ASRCInQueueIdle;
    }

    if (asrcHandle->callback)
    {
        (asrcHandle->callback)(privHandle->base, asrcHandle, inStatus, asrcHandle->userData);
    }
}

/*!
 * brief Initializes the ASRC P2P eDMA handle.
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
void ASRC_TransferCreateP2PHandleEDMA(ASRC_Type *base,
                                      asrc_p2p_edma_handle_t *handle,
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
    EDMA_SetCallback(outDmaHandle, NULL, &s_edmaPrivateHandle[instance][channelPair]);
}

/*!
 * brief Configures the ASRC P2P channel pair.
 *
 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 * param asrcConfig asrc configurations.
 * param periphConfig peripheral configuration.
 * param inputSampleRate ASRC input sample rate.
 * param outputSampleRate ASRC output sample rate.
 */
status_t ASRC_TransferSetChannelPairP2PConfigEDMA(ASRC_Type *base,
                                                  asrc_p2p_edma_handle_t *handle,
                                                  asrc_channel_pair_config_t *asrcConfig,
                                                  const asrc_p2p_edma_config_t *periphConfig,
                                                  uint32_t inSampleRate,
                                                  uint32_t outSampleRate)
{
    assert(handle && asrcConfig);

    /* Configure the audio format to ASRC registers */
    if (ASRC_SetChannelPairConfig(base, handle->channelPair, asrcConfig, inSampleRate, outSampleRate))
    {
        return kStatus_ASRCChannelPairConfigureFailed;
    }
    handle->out.peripheralConfig = periphConfig;
    handle->in.fifoThreshold     = asrcConfig->inFifoThreshold * asrcConfig->audioDataChannels;
    handle->out.fifoThreshold    = asrcConfig->outFifoThreshold * asrcConfig->audioDataChannels;
    ASRC_MapSamplesWidth(base, handle->channelPair, &handle->in.sampleWidth, &handle->out.sampleWidth);

    return kStatus_Success;
}

/*!
 * brief Get P2P output samples size.
 *
 * Used to get the samples size that is aligned with fifo threshold.
 *
 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 * param inputSampleRate ASRC input sample rate.
 * param outputSampleRate ASRC output sample rate.
 * param inSamples input samples number.
 * retval output samples size in byte.
 */
uint32_t ASRC_TransferGetP2POutSamplesSizeEDMA(
    ASRC_Type *base, asrc_p2p_edma_handle_t *handle, uint32_t inSampleRate, uint32_t outSampleRate, uint32_t inSamples)
{
    uint32_t outputSize = ASRC_GetOutSamplesSize(base, handle->channelPair, inSampleRate, outSampleRate, inSamples);

    return outputSize - (outputSize % (handle->out.fifoThreshold * handle->out.sampleWidth));
}

/*!
 * brief Performs a non-blocking ASRC P2P convert using EDMA.
 *
 * note This interface returns immediately after the transfer initiates.

 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 * param inDataAddr input buffer address.
 * param inDataSize input data size.
 * param outDataSize output data size.
 * retval kStatus_Success Start a ASRC eDMA send successfully.
 * retval kStatus_InvalidArgument The input argument is invalid.
 * retval kStatus_ASRCQueueFull ASRC EDMA driver queue is full.
 */
status_t ASRC_TransferP2PEDMA(
    ASRC_Type *base, asrc_p2p_edma_handle_t *handle, void *inDataAddr, uint32_t inDataSize, uint32_t outDataSize)
{
    assert(handle != NULL);

    edma_transfer_config_t config = {0};
    uint32_t inAddr               = ASRC_GetInputDataRegisterAddress(base, handle->channelPair);
    uint32_t outAddr              = ASRC_GetOutputDataRegisterAddress(base, handle->channelPair);
    uint32_t periphFifoAddr = handle->out.peripheralConfig->getStartFifoAddr(handle->out.peripheralConfig->channel);

    /* Check if input parameter invalid */
    if ((inDataAddr == NULL) || (inDataSize == 0U) || (outDataSize == 0U))
    {
        return kStatus_InvalidArgument;
    }

    if ((handle->in.asrcQueue[handle->in.queueUser]))
    {
        return kStatus_ASRCQueueFull;
    }

    /* Add into queue */
    handle->in.asrcQueue[handle->in.queueUser]    = inDataAddr;
    handle->in.transferSize[handle->in.queueUser] = inDataSize;
    handle->in.queueUser                          = (handle->in.queueUser + 1) % ASRC_XFER_QUEUE_SIZE;

    /* Prepare ASRC input edma configuration */
    EDMA_PrepareTransfer(&config, (void *)outAddr, handle->out.sampleWidth, (void *)periphFifoAddr,
                         handle->out.sampleWidth, handle->out.fifoThreshold * handle->out.sampleWidth, outDataSize,
                         kEDMA_PeripheralToPeripheral);
    if (EDMA_SubmitTransfer(handle->out.outDmaHandle, &config) != kStatus_Success)
    {
        return kStatus_ASRCQueueFull;
    }

    /* Prepare ASRC output edma configuration */
    EDMA_PrepareTransfer(&config, inDataAddr, handle->in.sampleWidth, (void *)inAddr, handle->in.sampleWidth,
                         handle->in.fifoThreshold * handle->in.sampleWidth, inDataSize, kEDMA_MemoryToPeripheral);
    EDMA_SubmitTransfer(handle->in.inDmaHandle, &config);

    if (handle->state != kStatus_ASRCBusy)
    {
        EDMA_StartTransfer(handle->out.outDmaHandle);
        EDMA_StartTransfer(handle->in.inDmaHandle);
        /* start peripheral */
        handle->out.peripheralConfig->startPeripheral(true);
    }

    return kStatus_Success;
}

/*!
 * brief Aborts a ASRC P2P transfer using eDMA.
 *
 * This function only aborts the current transfer slots, the other transfer slots' information still kept
 * in the handler. If users want to terminate all transfer slots, just call ASRC_TransferTerminalP2PEDMA.
 *
 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferAbortP2PEDMA(ASRC_Type *base, asrc_p2p_edma_handle_t *handle)
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
 * current transfer slot, please call ASRC_TransferAbortPP2PEDMA.
 *
 * param base ASRC base pointer.
 * param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferTerminalP2PEDMA(ASRC_Type *base, asrc_p2p_edma_handle_t *handle)
{
    assert(handle);

    /* Abort the current transfer */
    ASRC_TransferAbortP2PEDMA(base, handle);
    /* start peripheral */
    handle->out.peripheralConfig->startPeripheral(true);

    /* Clear all the internal information */
    memset(handle->in.tcd, 0U, sizeof(handle->in.tcd));
    memset(handle->in.asrcQueue, 0U, sizeof(handle->in.asrcQueue));
    memset(handle->in.transferSize, 0U, sizeof(handle->in.transferSize));
    handle->in.queueUser   = 0U;
    handle->in.queueDriver = 0U;

    memset(handle->out.tcd, 0U, sizeof(handle->out.tcd));
}
