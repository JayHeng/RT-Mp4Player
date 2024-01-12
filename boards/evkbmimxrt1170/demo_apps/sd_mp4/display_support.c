/*
 * Copyright 2019-2021, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "display_support.h"
#include "fsl_gpio.h"
#include "mp4.h"
#include "fsl_mipi_dsi.h"
#if (DEMO_PANEL_RK055AHD091 == DEMO_PANEL)
#include "fsl_rm68200.h"
#elif (DEMO_PANEL_RK055IQH091 == DEMO_PANEL)
#include "fsl_rm68191.h"
#elif (DEMO_PANEL_RK055MHD091 == DEMO_PANEL)
#include "fsl_hx8394.h"
#elif (DEMO_PANEL_G1120B0MIPI == DEMO_PANEL)
#include "fsl_dc_fb_dsi_cmd.h"
#include "fsl_rm67162.h"
#include "mipi_dsi_aux.h"
//#include "auo141_display.h"
#endif
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
#include "fsl_dc_fb_lcdifv2.h"
#else
#include "fsl_dc_fb_elcdif.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*
 * RK055AHD091 panel
 */

#if (DEMO_PANEL == DEMO_PANEL_RK055AHD091)
#define DEMO_HSW 8
#define DEMO_HFP 32
#define DEMO_HBP 32
#define DEMO_VSW 2
#define DEMO_VFP 16
#define DEMO_VBP 14

#elif (DEMO_PANEL_RK055IQH091 == DEMO_PANEL)

#define DEMO_HSW 2
#define DEMO_HFP 32
#define DEMO_HBP 30
#define DEMO_VSW 2
#define DEMO_VFP 16
#define DEMO_VBP 14

#elif (DEMO_PANEL_RK055MHD091 == DEMO_PANEL)

#define DEMO_HSW 6
#define DEMO_HFP 12
#define DEMO_HBP 24
#define DEMO_VSW 2
#define DEMO_VFP 16
#define DEMO_VBP 14

#elif (DEMO_PANEL_G1120B0MIPI == DEMO_PANEL)

#define DEMO_HSW 0
#define DEMO_HFP 0
#define DEMO_HBP 0
#define DEMO_VSW 0
#define DEMO_VFP 0
#define DEMO_VBP 0

#endif

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)

#define DEMO_LCDIF_POL_FLAGS                                                             \
    (kLCDIFV2_DataEnableActiveHigh | kLCDIFV2_VsyncActiveLow | kLCDIFV2_HsyncActiveLow | \
     kLCDIFV2_DriveDataOnFallingClkEdge)

#define DEMO_LCDIF LCDIFV2

#else

#define DEMO_LCDIF_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnFallingClkEdge)

#define DEMO_LCDIF LCDIF

#endif

/* Definitions for MIPI. */
#define DEMO_MIPI_DSI          (&g_mipiDsi)
#if (DEMO_PANEL_G1120B0MIPI == DEMO_PANEL)
#define DEMO_MIPI_DSI_LANE_NUM 1
#else
#define DEMO_MIPI_DSI_LANE_NUM 2
#endif


/*
 * The DPHY bit clock must be fast enough to send out the pixels, it should be
 * larger than:
 *
 *         (Pixel clock * bit per output pixel) / number of MIPI data lane
 *
 * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
 * it is fast enough.
 */
#define DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(origin) (((origin) / 8) * 9)
      

/*
 * The max TX array size:
 *
 * 1. One byte in FIFO is reserved for DSC command
 * 2. One pixel should not be split to two transfer.
 */
#define APP_DSI_TX_ARRAY_MAX   (((FSL_DSI_TX_MAX_PAYLOAD_BYTE - 1U) / DEMO_BUFFER_BYTE_PER_PIXEL) * DEMO_BUFFER_BYTE_PER_PIXEL)

typedef struct _dsi_mem_write_ctx
{
    volatile bool onGoing;
    const uint8_t *txData;
    uint32_t leftByteLen;
    uint8_t dscCmd;
} dsi_mem_write_ctx_t;

void BOARD_InitMipiPanelTEPin(void);
status_t BOARD_DSI_MemWrite(uint8_t virtualChannel, const uint8_t *data, uint32_t length);

static dsi_mem_write_ctx_t s_dsiMemWriteCtx;
static dsi_transfer_t      s_dsiMemWriteXfer = {0};
static dsi_handle_t        s_dsiDriverHandle;
static uint8_t             s_dsiMemWriteTmpArray[APP_DSI_TX_ARRAY_MAX];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void BOARD_PullPanelResetPin(bool pullUp);
static void BOARD_PullPanelPowerPin(bool pullUp);
void BOARD_InitLcdifClock(void);
static void BOARD_InitMipiDsiClock(void);
static status_t BOARD_DSI_Transfer(dsi_transfer_t *xfer);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint32_t mipiDsiTxEscClkFreq_Hz;
static uint32_t mipiDsiDphyBitClkFreq_Hz;
static uint32_t mipiDsiDphyRefClkFreq_Hz;
static uint32_t mipiDsiDpiClkFreq_Hz;

const MIPI_DSI_Type g_mipiDsi = {
    .host = DSI_HOST,
    .apb  = DSI_HOST_APB_PKT_IF,
    .dpi  = DSI_HOST_DPI_INTFC,
    .dphy = DSI_HOST_DPHY_INTFC,
};

#if (DEMO_PANEL == DEMO_PANEL_RK055AHD091)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = BOARD_DSI_Transfer,
};

static const rm68200_resource_t rm68200Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t rm68200Handle = {
    .resource = &rm68200Resource,
    .ops      = &rm68200_ops,
};

#elif (DEMO_PANEL == DEMO_PANEL_RK055MHD091)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = BOARD_DSI_Transfer,
};

static const hx8394_resource_t hx8394Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t hx8394Handle = {
    .resource = &hx8394Resource,
    .ops      = &hx8394_ops,
};

#elif (DEMO_PANEL == DEMO_PANEL_RK055IQH091)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = BOARD_DSI_Transfer,
};

static const rm68191_resource_t rm68191Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t rm68191Handle = {
    .resource = &rm68191Resource,
    .ops      = &rm68191_ops,
};

#elif (DEMO_PANEL == DEMO_PANEL_G1120B0MIPI)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = BOARD_DSI_Transfer,
    .memWriteFunc   = BOARD_DSI_MemWrite,
};

static const rm67162_resource_t rm67162Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t rm67162Handle = {
    .resource = &rm67162Resource,
    .ops      = &rm67162_ops,
};

#endif

#if (DEMO_PANEL == DEMO_PANEL_G1120B0MIPI)

const dc_fb_dsi_cmd_config_t s_panelConfig = {
    .commonConfig =
        {
            .resolution   = FSL_VIDEO_RESOLUTION(DEMO_PANEL_WIDTH, DEMO_PANEL_HEIGHT),
            .hsw          = DEMO_HSW,
            .hfp          = DEMO_HFP,
            .hbp          = DEMO_HBP,
            .vsw          = DEMO_VSW,
            .vfp          = DEMO_VFP,
            .vbp          = DEMO_VBP,
            .controlFlags = 0,
            .dsiLanes     = DEMO_MIPI_DSI_LANE_NUM,
            .pixelFormat  = APP_VIDEO_PIXEL_FORMAT,
        },
    .useTEPin = true,
};

static dc_fb_dsi_cmd_handle_t s_dcFbDsiCmdHandle = {
    .dsiDevice   = &dsiDevice,
    .panelHandle = &rm67162Handle,
};

const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsDsiCmd,
    .prvData = &s_dcFbDsiCmdHandle,
    .config  = &s_panelConfig,
};
#else
#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)

static dc_fb_lcdifv2_handle_t s_dcFbLcdifv2Handle = {0};

static const dc_fb_lcdifv2_config_t s_dcFbLcdifv2Config = {
    .lcdifv2       = DEMO_LCDIF,
    .width         = DEMO_PANEL_WIDTH,
    .height        = DEMO_PANEL_HEIGHT,
    .hsw           = DEMO_HSW,
    .hfp           = DEMO_HFP,
    .hbp           = DEMO_HBP,
    .vsw           = DEMO_VSW,
    .vfp           = DEMO_VFP,
    .vbp           = DEMO_VBP,
    .polarityFlags = DEMO_LCDIF_POL_FLAGS,
    .lineOrder     = kLCDIFV2_LineOrderRGB,
/* CM4 is domain 1, CM7 is domain 0. */
#if (__CORTEX_M <= 4)
    .domain = 1,
#else
    .domain = 0,
#endif
};

const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsLcdifv2,
    .prvData = &s_dcFbLcdifv2Handle,
    .config  = &s_dcFbLcdifv2Config,
};

#else

dc_fb_elcdif_handle_t s_dcFbElcdifHandle = {0}; /* The handle must be initialized to 0. */

const dc_fb_elcdif_config_t s_dcFbElcdifConfig = {
    .elcdif        = DEMO_LCDIF,
    .width         = DEMO_PANEL_WIDTH,
    .height        = DEMO_PANEL_HEIGHT,
    .hsw           = DEMO_HSW,
    .hfp           = DEMO_HFP,
    .hbp           = DEMO_HBP,
    .vsw           = DEMO_VSW,
    .vfp           = DEMO_VFP,
    .vbp           = DEMO_VBP,
    .polarityFlags = DEMO_LCDIF_POL_FLAGS,
#if (!DEMO_USE_XRGB8888) && (DEMO_USE_LUT8)
    .dataBus       = kELCDIF_DataBus8Bit,
#else
    .dataBus       = kELCDIF_DataBus24Bit,
#endif
};

const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsElcdif,
    .prvData = &s_dcFbElcdifHandle,
    .config  = &s_dcFbElcdifConfig,
};
#endif
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

static void BOARD_PullPanelResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 0);
    }
}

static void BOARD_PullPanelPowerPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 0);
    }
}

static status_t BOARD_DSI_Transfer(dsi_transfer_t *xfer)
{
    return DSI_TransferBlocking(DEMO_MIPI_DSI, xfer);
}

void BOARD_InitLcdifClock(void)
{
    /*
     * The pixel clock is (height + VSW + VFP + VBP) * (width + HSW + HFP + HBP) * frame rate.
     *
     * For 60Hz frame rate, the RK055IQH091 pixel clock should be 36MHz.
     * the RK055AHD091 pixel clock should be 62MHz.
     */
    const clock_root_config_t lcdifClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
#if ((DEMO_PANEL == DEMO_PANEL_RK055AHD091) || (DEMO_PANEL_RK055MHD091 == DEMO_PANEL))
#if VIDEO_LCD_REFRESH_FREG_60Hz == 1
        .div = 8,
#elif VIDEO_LCD_REFRESH_FREG_30Hz == 1
        .div = 17,
#elif VIDEO_LCD_REFRESH_FREG_25Hz == 1
        .div = 20,
#endif
#else
#if VIDEO_LCD_REFRESH_FREG_60Hz == 1
        .div = 14,
#elif VIDEO_LCD_REFRESH_FREG_30Hz == 1
        .div = 29,
#elif VIDEO_LCD_REFRESH_FREG_25Hz == 1
        .div = 35,
#endif
#endif
    };

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    CLOCK_SetRootClock(kCLOCK_Root_Lcdifv2, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdifv2);

#else

    CLOCK_SetRootClock(kCLOCK_Root_Lcdif, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdif);
#endif
}

#if (DEMO_PANEL == DEMO_PANEL_G1120B0MIPI)
static void BOARD_InitMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 528MHz / 11 = 48MHz. */
    /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
        .div      = 11,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false, .resetDiv = 2, .div0 = 2, /* TX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
    mipiDsiDphyBitClkFreq_Hz = BOARD_MIPI_CLK_HZ;

}

static void BOARD_SetMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes       = DEMO_MIPI_DSI_LANE_NUM;
    dsiConfig.autoInsertEoTp = true;

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    /* Init the DSI module. */
    DSI_Init(DEMO_MIPI_DSI, &dsiConfig);

    /* Init DPHY */
    DSI_InitDphy(DEMO_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);
}
#else

static void BOARD_InitMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 528MHz / 11 = 48MHz. */
    /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
        .div      = 11,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false, .resetDiv = 2, .div0 = 2, /* TX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
}

static void BOARD_SetMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    const dsi_dpi_config_t dpiConfig = {.pixelPayloadSize = DEMO_PANEL_WIDTH,
                                        .dpiColorCoding   = kDSI_Dpi24Bit,
                                        .pixelPacket      = kDSI_PixelPacket24Bit,
                                        .videoMode        = kDSI_DpiBurst,
                                        .bllpMode         = kDSI_DpiBllpLowPower,
                                        .polarityFlags    = kDSI_DpiVsyncActiveLow | kDSI_DpiHsyncActiveLow,
                                        .hfp              = DEMO_HFP,
                                        .hbp              = DEMO_HBP,
                                        .hsw              = DEMO_HSW,
                                        .vfp              = DEMO_VFP,
                                        .vbp              = DEMO_VBP,
                                        .panelHeight      = DEMO_PANEL_HEIGHT,
                                        .virtualChannel   = 0};

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes       = DEMO_MIPI_DSI_LANE_NUM;
    dsiConfig.autoInsertEoTp = true;

    /* Init the DSI module. */
    DSI_Init(DEMO_MIPI_DSI, &dsiConfig);

    /* Init DPHY.
     *
     * The DPHY bit clock must be fast enough to send out the pixels, it should be
     * larger than:
     *
     *         (Pixel clock * bit per output pixel) / number of MIPI data lane
     *
     * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
     * it is fast enough.
     *
     * Note that the DSI output pixel is 24bit per pixel.
     */
    mipiDsiDphyBitClkFreq_Hz = mipiDsiDpiClkFreq_Hz * (24 / DEMO_MIPI_DSI_LANE_NUM);

    mipiDsiDphyBitClkFreq_Hz = DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(mipiDsiDphyBitClkFreq_Hz);

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    mipiDsiDphyBitClkFreq_Hz = DSI_InitDphy(DEMO_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);

    /* Init DPI interface. */
    DSI_SetDpiConfig(DEMO_MIPI_DSI, &dpiConfig, DEMO_MIPI_DSI_LANE_NUM, mipiDsiDpiClkFreq_Hz, mipiDsiDphyBitClkFreq_Hz);
}

#endif

static status_t BOARD_InitLcdPanel(void)
{
    status_t status = kStatus_Success;

    const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    const display_config_t displayConfig = {
        .resolution   = FSL_VIDEO_RESOLUTION(DEMO_PANEL_WIDTH, DEMO_PANEL_HEIGHT),
        .hsw          = DEMO_HSW,
        .hfp          = DEMO_HFP,
        .hbp          = DEMO_HBP,
        .vsw          = DEMO_VSW,
        .vfp          = DEMO_VFP,
        .vbp          = DEMO_VBP,
        .controlFlags = 0,
        .dsiLanes     = DEMO_MIPI_DSI_LANE_NUM,
    };

    GPIO_PinInit(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, &pinConfig);

#if (DEMO_PANEL == DEMO_PANEL_RK055AHD091)
    status = RM68200_Init(&rm68200Handle, &displayConfig);
#elif (DEMO_PANEL_RK055MHD091 == DEMO_PANEL)
    status = HX8394_Init(&hx8394Handle, &displayConfig);
#elif (DEMO_PANEL_RK055IQH091 == DEMO_PANEL)
    status = RM68191_Init(&rm68191Handle, &displayConfig);
#elif (DEMO_PANEL_G1120B0MIPI == DEMO_PANEL)
    BOARD_InitMipiPanelTEPin();
#endif

    if (status == kStatus_Success)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, 1);
    }

    return status;
}

status_t BOARD_InitDisplayInterface(void)
{
    CLOCK_EnableClock(kCLOCK_Video_Mux);

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    /* LCDIF v2 output to MIPI DSI. */
    VIDEO_MUX->VID_MUX_CTRL.SET = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#else
    /* ELCDIF output to MIPI DSI. */
    VIDEO_MUX->VID_MUX_CTRL.CLR = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#endif

    /* 1. Power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /* 2. Assert MIPI reset. */
    IOMUXC_GPR->GPR62 &=
        ~(IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK |
          IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 3. Setup clock. */
    BOARD_InitMipiDsiClock();

    /* 4. Deassert PCLK and ESC reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK);

    /* 5. Configures peripheral. */
    BOARD_SetMipiDsiConfig();

    /* 6. Deassert BYTE and DBI reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 7. Configure the panel. */
    return BOARD_InitLcdPanel();
}

#if (DEMO_PANEL_G1120B0MIPI == DEMO_PANEL)
static status_t BOARD_DsiMemWriteSendChunck(void)
{
    uint32_t curSendLen;
    uint32_t i;

    curSendLen =
        APP_DSI_TX_ARRAY_MAX > s_dsiMemWriteCtx.leftByteLen ? s_dsiMemWriteCtx.leftByteLen : APP_DSI_TX_ARRAY_MAX;

    s_dsiMemWriteXfer.txDataType = kDSI_TxDataDcsLongWr;
    s_dsiMemWriteXfer.dscCmd     = s_dsiMemWriteCtx.dscCmd;
    s_dsiMemWriteXfer.txData     = s_dsiMemWriteTmpArray;

    /* For each pixel, the MIPI DSI sends out low byte first, but according to
     * the MIPI DSC spec, the high byte should be send first, so swap the pixel byte
     * first.
     */
#if (DEMO_RM67162_BUFFER_FORMAT == DEMO_RM67162_BUFFER_RGB565)
        s_dsiMemWriteXfer.txDataSize = curSendLen;
    for (i = 0; i < curSendLen; i += 2)
    {
        s_dsiMemWriteTmpArray[i]     = *(s_dsiMemWriteCtx.txData + 1);
        s_dsiMemWriteTmpArray[i + 1] = *(s_dsiMemWriteCtx.txData);

        s_dsiMemWriteCtx.txData += 2;
    }
#elif (DEMO_RM67162_BUFFER_FORMAT == DEMO_RM67162_BUFFER_RGB888)
    s_dsiMemWriteXfer.txDataSize = curSendLen;
    for (i = 0; i < curSendLen; i += 3)
    {
        s_dsiMemWriteTmpArray[i]     = *(s_dsiMemWriteCtx.txData + 2);
        s_dsiMemWriteTmpArray[i + 1] = *(s_dsiMemWriteCtx.txData + 1);
        s_dsiMemWriteTmpArray[i + 2] = *(s_dsiMemWriteCtx.txData);

        s_dsiMemWriteCtx.txData += 3;
    }
#else
    s_dsiMemWriteXfer.txDataSize = (curSendLen * 3 / 4);
    for (i = 0; i < (curSendLen * 3 / 4); i += 3)
    {
        s_dsiMemWriteTmpArray[i]     = *(s_dsiMemWriteCtx.txData + 2);
        s_dsiMemWriteTmpArray[i + 1] = *(s_dsiMemWriteCtx.txData + 1);
        s_dsiMemWriteTmpArray[i + 2] = *(s_dsiMemWriteCtx.txData);

        s_dsiMemWriteCtx.txData += 4;
    }
#endif

    s_dsiMemWriteCtx.leftByteLen -= (curSendLen);
    s_dsiMemWriteCtx.dscCmd = kMIPI_DCS_WriteMemoryContinue;

    return DSI_TransferNonBlocking(DEMO_MIPI_DSI, &s_dsiDriverHandle, &s_dsiMemWriteXfer);
}

static void BOARD_DsiMemWriteCallback(const MIPI_DSI_Type *base, dsi_handle_t *handle, status_t status, void *userData)
{
    if ((kStatus_Success == status) && (s_dsiMemWriteCtx.leftByteLen > 0))
    {
        status = BOARD_DsiMemWriteSendChunck();

        if (kStatus_Success == status)
        {
            return;
        }
    }

    s_dsiMemWriteCtx.onGoing = false;
    MIPI_DSI_MemoryDoneDriverCallback(status, &dsiDevice);
}

status_t BOARD_DSI_MemWrite(uint8_t virtualChannel, const uint8_t *data, uint32_t length)
{
    status_t status;

    if (s_dsiMemWriteCtx.onGoing)
    {
        return kStatus_Fail;
    }

    s_dsiMemWriteXfer.virtualChannel = virtualChannel;
    s_dsiMemWriteXfer.flags          = kDSI_TransferUseHighSpeed;
    s_dsiMemWriteXfer.sendDscCmd     = true;

    s_dsiMemWriteCtx.onGoing     = true;
    s_dsiMemWriteCtx.txData      = data;
    s_dsiMemWriteCtx.leftByteLen = length;
    s_dsiMemWriteCtx.dscCmd      = kMIPI_DCS_WriteMemoryStart;

    status = BOARD_DsiMemWriteSendChunck();

    if (status != kStatus_Success)
    {
        /* Memory write does not start actually. */
        s_dsiMemWriteCtx.onGoing = false;
    }

    return status;
}

void BOARD_InitMipiPanelTEPin(void)
{
    const gpio_pin_config_t tePinConfig = {
        .direction = kGPIO_DigitalInput,
        .outputLogic  = 0,
        .interruptMode = kGPIO_IntRisingEdge,
    };

    /*
     * TE pin configure method:
     *
     * The TE pin interrupt is like this:
     *
     *            VSYNC
     *         +--------+
     *         |        |
     *         |        |
     * --------+        +----------------
     *
     * 1. If one frame send time is shorter than one frame refresh time, then set
     *    TE pin interrupt at the start of VSYNC.
     * 2. If one frame send time is longer than one frame refresh time, and shorter
     *    than two frames refresh time, then set TE pin interrupt at the end of VSYNC.
     * 3. If one frame send time is longer than two frame refresh time, tearing effect
     *    could not be removed.
     *
     * For RM67162 @60Hz frame rate in single core version, frame refresh time is 16.7 ms. After test,
     * one frame send time is shorter than one frame refresh time. So TE interrupt is
     * set to start of VSYNC.
     */

    GPIO_PinInit(BOARD_MIPI_PANEL_TE_GPIO, BOARD_MIPI_PANEL_TE_PIN, &tePinConfig);

    GPIO_PortClearInterruptFlags(BOARD_MIPI_PANEL_TE_GPIO, 1<<BOARD_MIPI_PANEL_TE_PIN);
    GPIO_PortEnableInterrupts(BOARD_MIPI_PANEL_TE_GPIO, 1<<BOARD_MIPI_PANEL_TE_PIN);

    NVIC_SetPriority(BOARD_MIPI_PANEL_TE_IRQ, 3);
    NVIC_EnableIRQ(BOARD_MIPI_PANEL_TE_IRQ);
}

status_t BOARD_PrepareDisplayController(void)
{
    status_t status;

    status = BOARD_InitDisplayInterface();
    if (kStatus_Success == status)
    {
        /*
         * Suggest setting to low priority. Because a new DSI transfer is prepared
         * in the callback BOARD_DsiMemWriteCallback, so the core spends more time
         * in ISR. Setting the low priority, then the important ISR won't be blocked.
         */
        NVIC_SetPriority(MIPI_DSI_IRQn, 6);
    }
    else
    {
        return status;
    }

    memset(&s_dsiMemWriteCtx, 0, sizeof(dsi_mem_write_ctx_t));

    /* Create the MIPI DSI trasnfer handle for non-blocking data trasnfer. */
    return DSI_TransferCreateHandle(DEMO_MIPI_DSI, &s_dsiDriverHandle, BOARD_DsiMemWriteCallback, NULL);
}

/* Smart panel TE pin IRQ handler. */
void BOARD_DisplayTEPinHandler(void)
{
    DC_FB_DSI_CMD_TE_IRQHandler(&g_dc);
}

#else
////////////////////////////////////////////////////////////////////////

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
void LCDIFv2_IRQHandler(void)
{
    DC_FB_LCDIFV2_IRQHandler(&g_dc);
}
#else
void eLCDIF_IRQHandler(void)
{
    DC_FB_ELCDIF_IRQHandler(&g_dc);
}
#endif

status_t BOARD_VerifyDisplayClockSource(void)
{
    status_t status;
    uint32_t srcClkFreq;

    /*
     * In this implementation, the SYSPLL2 (528M) clock is used as the source
     * of LCDIFV2 pixel clock and MIPI DSI ESC clock. The OSC24M clock is used
     * as the MIPI DSI DPHY PLL reference clock. This function checks the clock
     * source are valid. OSC24M is always valid, so only verify the SYSPLL2.
     */
    srcClkFreq = CLOCK_GetPllFreq(kCLOCK_PllSys2);
    if (528 != (srcClkFreq / 1000000))
    {
        status = kStatus_Fail;
    }
    else
    {
        status = kStatus_Success;
    }

    return status;
}

status_t BOARD_PrepareDisplayController(void)
{
    status_t status;

    status = BOARD_VerifyDisplayClockSource();

    if (status != kStatus_Success)
    {
        PRINTF("Error: Invalid display clock source.\r\n");
        return status;
    }

    BOARD_InitLcdifClock();

    status = BOARD_InitDisplayInterface();

    if (kStatus_Success == status)
    {
#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
        NVIC_ClearPendingIRQ(LCDIFv2_IRQn);
        NVIC_SetPriority(LCDIFv2_IRQn, 3);
        EnableIRQ(LCDIFv2_IRQn);
#else
        NVIC_ClearPendingIRQ(eLCDIF_IRQn);
        NVIC_SetPriority(eLCDIF_IRQn, 3);
        EnableIRQ(eLCDIF_IRQn);
#endif
    }

    return kStatus_Success;
}
#endif
