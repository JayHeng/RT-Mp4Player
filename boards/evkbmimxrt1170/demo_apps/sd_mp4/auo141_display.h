/*
 * auo_display.h
 *
 *  Created on: 2022年2月28日
 *      Author: KanC
 */

#ifndef AUO141_DISPLAY_H_
#define AUO141_DISPLAY_H_

#include "fsl_display.h"
#include "fsl_mipi_dsi_cmd.h"


typedef struct _auo141_resource
{
    mipi_dsi_device_t *dsiDevice;      /*!< MIPI DSI device. */
    void (*pullResetPin)(bool pullUp); /*!< Function to pull reset pin high or low. */
    void (*pullPowerPin)(bool pullUp); /*!< Function to pull power pin high or low. */
} auo141_resource_t;

extern const display_operations_t auo141_ops;

status_t AUO141_Init(display_handle_t *handle, const display_config_t *config);

status_t AUO141_Deinit(display_handle_t *handle);

status_t AUO141_Start(display_handle_t *handle);

status_t AUO141_Stop(display_handle_t *handle);

#endif /* AUO141_DISPLAY_H_ */
