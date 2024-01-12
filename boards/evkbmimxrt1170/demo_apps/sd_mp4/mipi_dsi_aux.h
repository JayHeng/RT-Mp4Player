/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MIPI_DSI_AUX_H_
#define _MIPI_DSI_AUX_H_

#include "fsl_mipi_dsi.h"

/*!
 * @addtogroup mipi_dsi_aux
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Forward declaration of the handle typedef. */
typedef struct _dsi_aux_handle dsi_aux_handle_t;

/*!
 * @brief MIPI DSI callback for finished transfer.
 *
 * When transfer finished, one of these status values will be passed to the user:
 * - @ref kStatus_Success Data transfer finished with no error.
 */
typedef void (*dsi_aux_callback_t)(const MIPI_DSI_Type *base,
                                        dsi_aux_handle_t *handle,
                                        status_t status,
                                        void *userData);

/*! @brief The pixel format feed Auxiliary core. */
typedef enum _dsi_aux_input_pixel_format
{
    kDSI_AUX_InputPixelFormatRGB565,   /*!< RGB565. */
    kDSI_AUX_InputPixelFormatRGB888,   /*!< RGB888. */
    kDSI_AUX_InputPixelFormatXRGB8888, /*!< XRGB8888. */
} dsi_aux_input_pixel_format_t;

/*! @brief The pixel format sent on MIPI DSI data lanes. */
typedef enum _dsi_aux_output_pixel_format
{
    kDSI_AUX_OutputPixelFormatRGB565, /*!< RGB565. */
    kDSI_AUX_OutputPixelFormatRGB888, /*!< RGB888. */
} dsi_aux_output_pixel_format_t;

/*! @brief The type definition for remote application event. */
typedef enum _dsi_aux_remote_app_event
{
    kDSI_AUX_RemoteCoreReady = 1, /*!< remote core ready */
    kDSI_AUX_ImageDataReadyForSent, /*!< data ready */
    kDSI_AUX_IMageDataSentdDone, /*!< data was sent. */
    kDSI_AUX_IMageDataSentdError /*!< data send failted. */
} dsi_aux_remote_app_event_t;

/*! @brief The pixel format sent on MIPI DSI data lanes. */
typedef struct _dsi_aux_write_mem_transfer
{
    dsi_aux_input_pixel_format_t inputFormat;   /*!< Input format. */
    dsi_aux_output_pixel_format_t outputFormat; /*!< Output format. */
    const uint8_t *data;                             /*!< Data to send. */
    size_t dataSize;                                 /*!< The byte count to be write. */
    uint8_t virtualChannel;                          /*!< Virtual channel used in the transfer,
                                                        current driver always use channel 0, added
                                                        for future enhancement. */

    /*!
     * If set to true, the pixels are filled to MIPI DSI FIFO directly.
     * If set to false, the pixel bytes are swapped then filled to
     * MIPI DSI FIFO. For example, when set to false and frame buffer pixel
     * format is RGB565:
     * LSB                                           MSB
     * B0 B1 B2 B3 B4 G0 G1 G2 | G3 G4 G5 R0 R1 R2 R3 R4
     * Then the pixel filled to DSI FIFO is:
     * LSB                                           MSB
     * G3 G4 G5 R0 R1 R2 R3 R4 | B0 B1 B2 B3 B4 G0 G1 G2
     */
    bool disablePixelByteSwap;
} dsi_aux_write_mem_transfer_t;

/*! @brief MIPI DSI transfer handle structure */
struct _dsi_aux_handle
{
    const MIPI_DSI_Type *dsi;          /*!< MIPI DSI peripheral. */
    volatile bool isBusy;             /*!< MIPI DSI is busy with data transfer. */
    dsi_aux_callback_t callback; /*!< DSI callback */
    void *userData;                   /*!< Callback parameter */
    
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Create the MIPI DSI AUX handle.
 *
 * @param base MIPI DSI host peripheral base address.
 * @param handle Handle pointer.
 * @param callback Callback function.
 * @param userData User data.
 */
status_t DSI_TransferCreateHandleAUX(const MIPI_DSI_Type *base,
                                          dsi_aux_handle_t *handle,
                                          dsi_aux_callback_t callback,
                                          void *userData);

/*!
 * @brief Write display controller video memory using Auxiliary core.
 *
 * Perform data transfer using Auxiliary core, when transfer finished,
 * upper layer could be informed through callback function.
 *
 * @param base MIPI DSI host peripheral base address.
 * @param handle pointer to dsi_aux_handle_t structure which stores the transfer state.
 * @param xfer Pointer to the transfer structure.
 *
 * @retval kStatus_Success Data transfer started successfully.
 * @retval kStatus_DSI_Busy Failed to start transfer because DSI is busy with pervious transfer.
 * @retval kStatus_DSI_NotSupported Transfer format not supported.
 */
status_t DSI_TransferWriteMemoryAUX(const MIPI_DSI_Type *base,
                                         dsi_aux_handle_t *handle,
                                         dsi_aux_write_mem_transfer_t *xfer);

/*!
 * @brief Abort current APB data transfer.
 *
 * @param base MIPI DSI host peripheral base address.
 * @param handle pointer to dsi_aux_handle_t structure which stores the transfer state.
 */
void DSI_TransferAbortAUX(const MIPI_DSI_Type *base, dsi_aux_handle_t *handle);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _MIPI_DSI_AUX_H_ */
