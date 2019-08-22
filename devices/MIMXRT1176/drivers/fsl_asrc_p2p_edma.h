/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FSL_ASRC_P2P_EDMA_H_
#define _FSL_ASRC_P2P_EDMA_H_

#include "fsl_edma.h"
#include "fsl_asrc.h"

/*!
 * @addtogroup asrc_p2p_edma_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_ASRC_EDMA_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

typedef struct _asrc_p2p_edma_handle asrc_p2p_edma_handle_t;

/*! @brief ASRC eDMA transfer callback function for finish and error */
typedef void (*asrc_edma_callback_t)(ASRC_Type *base, asrc_p2p_edma_handle_t *handle, status_t status, void *userData);

/*! @brief ASRC trigger peripheral function pointer */
typedef void (*asrc_start_peripheral_t)(bool start);
/*! @brief ASRC get peripheral fifo address */
typedef uint32_t (*asrc_get_peripheralFifoAddr_t)(uint32_t channel);
/*! @brief destination peripheral configuration */
typedef struct _asrc_p2p_edma_config
{
    uint8_t watermark;                              /*!< peripheral watermark */
    uint8_t channel;                                /*!< peripheral channel number */
    asrc_start_peripheral_t startPeripheral;        /*!< trigger peripheral start */
    asrc_get_peripheralFifoAddr_t getStartFifoAddr; /*!< get fifo address */
} asrc_p2p_edma_config_t;

/*!@ brief asrc in edma handler */
typedef struct _asrc_p2p_in_edma_handle
{
    edma_handle_t *inDmaHandle;                                    /*!< DMA handler for ASRC in */
    uint8_t tcd[(ASRC_XFER_QUEUE_SIZE + 1U) * sizeof(edma_tcd_t)]; /*!< TCD pool for eDMA send. */
    uint32_t sampleWidth;                                          /*!< input data width */
    uint32_t fifoThreshold;                                        /*!< ASRC input fifo threshold */
    uint32_t *asrcQueue[ASRC_XFER_QUEUE_SIZE];                     /*!< Transfer queue storing queued transfer. */
    size_t transferSize[ASRC_XFER_QUEUE_SIZE];                     /*!< Data bytes need to transfer */
    volatile uint8_t queueUser;                                    /*!< Index for user to queue transfer. */
    volatile uint8_t queueDriver; /*!< Index for driver to get the transfer data and size */
} asrc_p2p_in_edma_handle_t;

/*!@ brief asrc out edma handler */
typedef struct _asrc_p2p_out_edma_handle
{
    edma_handle_t *outDmaHandle;                                   /*!< DMA handler for ASRC out */
    uint8_t tcd[(ASRC_XFER_QUEUE_SIZE + 1U) * sizeof(edma_tcd_t)]; /*!< TCD pool for eDMA send. */
    uint32_t sampleWidth;                                          /*!< output data width */
    uint32_t fifoThreshold;                                        /*!< ASRC output fifo threshold */
    const asrc_p2p_edma_config_t *peripheralConfig;                /*!< peripheral configuration pointer */
} asrc_p2p_out_edma_handle_t;

/*! @brief ASRC DMA transfer handle.*/
struct _asrc_p2p_edma_handle
{
    asrc_p2p_in_edma_handle_t in;    /*!< asrc in handler */
    asrc_p2p_out_edma_handle_t out;  /*!< asrc out handler */
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
 * @brief Initializes the ASRC p2p eDMA handle.
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
void ASRC_TransferCreateP2PHandleEDMA(ASRC_Type *base,
                                      asrc_p2p_edma_handle_t *handle,
                                      asrc_channel_pair_t channelPair,
                                      asrc_edma_callback_t callback,
                                      edma_handle_t *txDmaHandle,
                                      edma_handle_t *rxDmaHandle,
                                      void *userData);

/*!
 * @brief Configures the ASRC P2P channel pair.
 *
 *
 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 * @param asrcConfig asrc configurations.
 * @param periphConfig peripheral configuration.
 * @param inputSampleRate ASRC input sample rate.
 * @param outputSampleRate ASRC output sample rate.
 */
status_t ASRC_TransferSetChannelPairP2PConfigEDMA(ASRC_Type *base,
                                                  asrc_p2p_edma_handle_t *handle,
                                                  asrc_channel_pair_config_t *asrcConfig,
                                                  const asrc_p2p_edma_config_t *periphConfig,
                                                  uint32_t inSampleRate,
                                                  uint32_t outSampleRate);

/*!
 * @brief Get P2P output samples size.
 *
 * Used to get the samples size that is aligned with fifo threshold.
 *
 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 * @param inputSampleRate ASRC input sample rate.
 * @param outputSampleRate ASRC output sample rate.
 * @param inSamples input samples number.
 * @retval output samples size in byte.
 */
uint32_t ASRC_TransferGetP2POutSamplesSizeEDMA(
    ASRC_Type *base, asrc_p2p_edma_handle_t *handle, uint32_t inSampleRate, uint32_t outSampleRate, uint32_t inSamples);

/*!
 * @brief Performs a non-blocking ASRC P2P convert using EDMA.
 *
 * @note This interface returns immediately after the transfer initiates.

 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 * @param inDataAddr input buffer address.
 * @param inDataSize input data size.
 * @param outDataSize output data size.
 * @retval  kStatus_Success Start a ASRC eDMA send successfully.
 *          kStatus_InvalidArgument The input argument is invalid.
 *          kStatus_ASRCQueueFull ASRC EDMA driver queue is full.
 */
status_t ASRC_TransferP2PEDMA(
    ASRC_Type *base, asrc_p2p_edma_handle_t *handle, void *inDataAddr, uint32_t inDataSize, uint32_t outDataSize);

/*!
 * @brief Aborts a ASRC P2P transfer using eDMA.
 *
 * This function only aborts the current transfer slots, the other transfer slots' information still kept
 * in the handler. If users want to terminate all transfer slots, just call ASRC_TransferTerminalP2PEDMA.
 *
 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferAbortP2PEDMA(ASRC_Type *base, asrc_p2p_edma_handle_t *handle);

/*!
 * @brief Terminate all ASRC Convert.
 *
 * This function will clear all transfer slots buffered in the asrc queue. If users only want to abort the
 * current transfer slot, please call ASRC_TransferAbortP2PEDMA.
 *
 * @param base ASRC base pointer.
 * @param handle ASRC eDMA handle pointer.
 */
void ASRC_TransferTerminalP2PEDMA(ASRC_Type *base, asrc_p2p_edma_handle_t *handle);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif
