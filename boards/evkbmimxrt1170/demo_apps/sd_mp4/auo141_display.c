/*
 * auo_display.c
 *
 *  Created on: 2022年2月28日
 *      Author: KanC
 */

#include "fsl_display.h"
#include "auo141_display.h"

#define AUO141_DelayMs VIDEO_DelayMs


const display_operations_t auo141_ops = {
    .init   = AUO141_Init,
    .deinit = AUO141_Deinit,
    .start  = AUO141_Start,
    .stop   = AUO141_Stop,
};

status_t AUO141_Init(display_handle_t *handle, const display_config_t *config)
{
    uint8_t param[6];
    status_t status                    = kStatus_Success;
    const auo141_resource_t *resource = (const auo141_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;

    /* Power on. */
//    resource->pullPowerPin(true);
//    AUO141_DelayMs(1);

    /* Perform reset. */
    resource->pullResetPin(false);
    AUO141_DelayMs(20);
    resource->pullResetPin(true);

    AUO141_DelayMs(120);

    /* For AAT1553A */
    param[0] = 0xFE;
    param[1] = 0x01;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}
    param[0] = 0xAA;
    param[1] = 0x4A;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}
    param[0] = 0x6A;
    param[1] = 0x4A;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0xFE;
    param[1] = 0x00;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}
#if 1
    param[0] = 0x2A;
    param[1] = 0x00;
    param[2] = 0x00;
    param[3] = 0x01;
    param[4] = 0x3F;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}
    param[0] = 0x2B;
    param[1] = 0x00;
    param[2] = 0x00;
    param[3] = 0x01;
    param[4] = 0x67;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x31;
    param[1] = 0x00;
    param[2] = 0x01;
    param[3] = 0x01;
    param[4] = 0x3F;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}
    param[0] = 0x30;
    param[1] = 0x00;
    param[2] = 0x01;
    param[3] = 0x01;
    param[4] = 0x67;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x12;
    status = MIPI_DSI_DCS_Write (dsiDevice, param, 1);
	if (kStatus_Success != status)
	{
		return status;
	}
	AUO141_DelayMs(5);

#endif

    param[0] = 0x35;
    param[1] = 0x02;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x53;
    param[1] = 0x20;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x51;
    param[1] = 0xFF;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x63;
    param[1] = 0xFF;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}

#if 0
    param[0] = 0x2A;
    param[1] = 0x00;
    param[2] = 0x00;
    param[3] = 0x01;
    param[4] = 0x3F;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x2B;
    param[1] = 0x00;
    param[2] = 0x00;
    param[3] = 0x01;
    param[4] = 0x67;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x30;
    param[1] = 0x00;
    param[2] = 0x01;
    param[3] = 0x01;
    param[4] = 0x67;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x31;
    param[1] = 0x00;
    param[2] = 0x01;
    param[3] = 0x01;
    param[4] = 0x3F;
    status = MIPI_DSI_GenericWrite(dsiDevice, param, 5);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x12;
    status = MIPI_DSI_DCS_Write (dsiDevice, param, 1);
	if (kStatus_Success != status)
	{
		return status;
	}
#endif

    param[0] = 0xFE;
    param[1] = 0x00;
    status = MIPI_DSI_DCS_Write(dsiDevice, param, 2);
	if (kStatus_Success != status)
	{
		return status;
	}

    param[0] = 0x11;
    status = MIPI_DSI_DCS_Write (dsiDevice, param, 1);
	if (kStatus_Success != status)
	{
		return status;
	}
	//AMOLED Display on.
	AUO141_DelayMs(150);

    param[0] = 0x29;
    status = MIPI_DSI_DCS_Write (dsiDevice, param, 1);
	if (kStatus_Success != status)
	{
		return status;
	}

    return kStatus_Success;
}

status_t AUO141_Deinit(display_handle_t *handle)
{
    const auo141_resource_t *resource = (const auo141_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;

    (void)MIPI_DSI_DCS_EnterSleepMode(dsiDevice, true);

    resource->pullResetPin(false);
    resource->pullPowerPin(false);

    return kStatus_Success;
}

status_t AUO141_Start(display_handle_t *handle)
{
    const auo141_resource_t *resource = (const auo141_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;

    return MIPI_DSI_DCS_SetDisplayOn(dsiDevice, true);
}

status_t AUO141_Stop(display_handle_t *handle)
{
    const auo141_resource_t *resource = (const auo141_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;

    return MIPI_DSI_DCS_SetDisplayOn(dsiDevice, false);
}
