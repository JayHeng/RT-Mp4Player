/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FSL_ASRC_M2M_EDMA_H_
#define _FSL_ASRC_M2M_EDMA_H_

#include "fsl_edma.h"
#include "fsl_asrc.h"

/*!
 * @addtogroup asrc_m2m_edma_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_ASRC_EDMA_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

typedef struct _asrc_edma_handle asrc_edma_handle_t;

/*! @brief ASRC eDMA transfer callback function for finish and error */
typedef void (*asrc_edma_callback_t)(ASRC_Type *base, asrc_edma_handle_t *handle, status_t status, void *userData);

/*!@ brief asrc in edma handler */
typedef struct _asrc_in_edma_handle
{
    edma_handle_t *inDmaHandle;                                    /*!< DMA handler for ASRC in */
    uint8_t tcd[(ASRC_XFER_QUEUE_SIZE + 1U) * sizeof(edma_tcd_t)]; /*!< TCD pool for eDMA send. */
    uint32_t sampleWidth;                                          /*!< input data width */
    uint32_t fifoThreshold;                                        /*!< ASRC input fifo threshold */
    uint8_t *asrcQueue[ASRC_XFER_QUEUE_SIZE];                      /*!< Transfer queue storing queued transfer. */
    size_t transferSize[ASRC_XFER_QUEUE_SIZE];                     /*!< Data bytes need to transfer */
    volatile uint8_t queueUser;                                    /*!< Index for user to queue transfer. */
    volatile uint8_t queueDriver; /*!< Index for driver to get the transfer data and size */
} asrc_in_edma_handle_t;

/*!@ brief asrc out edma handler */
typedef struct _asrc_out_edma_handle
{
    edma_handle_t *outDmaHandle;                                   /*!< DMA handler for ASRC out */
    uint8_t tcd[(ASRC_XFER_QUEUE_SIZE + 1U) * sizeof(edma_tcd_t)]; /*!< TCD pool for eDMA send. */
    uint32_t sampleWidth;                                          /*!< output data width */
    uint32_t fifoThreshold;                                        /*!< ASRC output fifo threshold */
    uint8_t *asrcQueue[ASRC_XFER_QUEUE_SIZE];                      /*!< Transfer queue storing queued transfer. */
    size_t transferSize[ASRC_XFER_QUEUE_SIZE];                     /*!< Data bytes need to transfer */
    volatile uint8_t queueUser;                                    /*!< Index for user to queue transfer. */
    volatile uint8_t queueDriver; /*!< Index for driver to get the transfer data and size */
    uint32_t pollingSize;         /*!< data size to polling out */
} asrc_out_edma_handle_t;

/*! @brief ASRC DMA transfer handle.*/
struct _asrc_edma_handle
{
    asrc_in_edma_handle_t in;        /*!< asrc in handler */
    asrc_out_edma_handle_t out;      /*!< asrc out handler */
    asrc_channel_pair_t channelPair; /*!< channel pair */
    uint32_t state;                  /*!< Internal state for ASRC eDMA transfer */
    void *userData;                  /*!< User callback parameter */
    asrc_edma_callback_t callback;   /*!< Callback for users while transfer finish or error occurs */
};

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name eDMA Transactional
 * @{
 */

/*!
 * @brief Initializes the ASRC m2m eDMA handle.
 *
 * This function initializes the ASRC DMA handle, which can be used for other ASRC transactional APIs.
 * Usually, for a specified ASRC channel pair, call this API once to get the initialized handle.
 *
 * @param base ASRC base pointer.
 * @param channelPair ASRC channel pair
 * @param handle ASRC eDMA handle pointer.
 * @param callback Pointer to user callback function.
 * @param txDmaHandle ASRC send edma handle pointer.
 * @param rxDmaHandle ASRC read edma handle pointer
 * @param userData User parameter passed to the callback function.
 */
void ASRC_TransferCreateHandleEDMA(ASRC_Type *base,
                                   asrc_edma_handle_t *handle,
                                   asrc_channel_pair_t channelPair,
                                   asrc_edma_callback_t callback,
                                   edma_handle_t *txDmaHandle,
                                   edma_handle_t *rxDmaHandle,
                                   void *userData);

/*!
 * @brief Configures the ASRC m2m channel pair.
 *
 *
 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 * @param asrcConfig asrc configurations.
 * @param inputSampleRate ASRC input sample rate.
 * @param outputSampleRate ASRC output sample rate.
 */
status_t ASRC_TransferSetChannelPairConfigEDMA(ASRC_Type *base,
                                               asrc_edma_handle_t *handle,
                                               asrc_channel_pair_config_t *asrcConfig,
                                               uint32_t inputSampleRate,
                                               uint32_t outputSampleRate);
/*!
 * @brief Performs a non-blocking ASRC m2m convert using EDMA.
 *
 * @note This interface returns immediately after the transfer initiates.

 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 * @param xfer Pointer to the DMA transfer structure.
 * @retval kStatus_Success Start a ASRC eDMA send successfully.
 * @retval kStatus_InvalidArgument The input argument is invalid.
 * @retval kStatus_ASRCQueueFull ASRC EDMA driver queue is full.
 */
status_t ASRC_TransferEDMA(ASRC_Type *base, asrc_edma_handle_t *handle, asrc_transfer_t *xfer);

/*!
 * @brief Terminate all ASRC Convert.
 *
 * This function will clear all transfer slots buffered in the asrc queue. If users only want to abort the
 * current transfer slot, please call ASRC_TransferAbortConvertEDMA.
 *
 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferTerminateEDMA(ASRC_Type *base, asrc_edma_handle_t *handle);

/*!
 * @brief Aborts a ASRC m2m transfer using eDMA.
 *
 * This function only aborts the current transfer slots, the other transfer slots' information still kept
 * in the handler. If users want to terminate all transfer slots, just call ASRC_TransferTerminateConvertEDMA.
 *
 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferAbortEDMA(ASRC_Type *base, asrc_edma_handle_t *handle);

/*!
 * @brief Gets byte count converted by ASRC.
 *
 * @param base ASRC base pointer
 * @param handle ASRC eDMA handle pointer.
 * @param count Bytes count received by ASRC.
 * @retval kStatus_Success Succeed get the transfer count.
 * @retval kStatus_ASRCIdle There is no non-blocking transaction in progress.
 */
status_t ASRC_TransferGetConvertedCountEDMA(ASRC_Type *base, asrc_edma_handle_t *handle, size_t *count);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif
