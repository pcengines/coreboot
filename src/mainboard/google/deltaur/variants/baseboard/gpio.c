/*
 * This file is part of the coreboot project.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <arch/acpi.h>
#include <baseboard/variants.h>
#include <baseboard/gpio.h>
#include <soc/gpio.h>
#include <variant/gpio.h>

static const struct pad_config gpio_table[] = {
	/* A0 thru A6 are ESPI, configured elsewhere */
	/* A0  : ESPI_IO0 ==> ESPI_IO_0 */
	/* A1  : ESPI_IO1 ==> ESPI_IO_1 */
	/* A2  : ESPI_IO2 ==> ESPI_IO_2 */
	/* A3  : ESPI_IO3 ==> ESPI_IO_3 */
	/* A4  : ESPI_CS# ==> ESPI_CS_L */
	/* A5  : ESPI_CLK ==> ESPI_CLK */
	/* A6  : ESPI_RESET# ==> NC(TP764) */
	/* A7  : GPP_A7 ==> CNVI_EN# */
	PAD_CFG_GPI(GPP_A7, NONE, DEEP),
	/* A8  : GPP_A8 ==> CNV_RF_RESET# */
	PAD_CFG_NF(GPP_A8, NONE, DEEP, NF2),
	/* A9  : GPP_A9 ==> CLKREQ_CNV#_1P8 */
	PAD_CFG_NF(GPP_A9, NONE, DEEP, NF2),
	/* A10 : GPP_A10 ==> TOUCH_SCREEN_RST# */
	PAD_CFG_GPO(GPP_A10, 0, DEEP),
	/* A11 : GPP_A11 ==> NC */
	PAD_NC(GPP_A11, NONE),
	/* A12 : GPP_A12 ==> M2280_PCIE_SATA# */
	PAD_CFG_NF(GPP_A12, NONE, DEEP, NF1),
	/* A13 : GPP_A13 ==> PCH_BT_RADIO_DIS# */
	PAD_CFG_GPO(GPP_A13, 0, DEEP),
	/* A14 : GPP_A14 ==> USB_OC1# */
	PAD_CFG_NF(GPP_A14, NONE, DEEP, NF1),
	/* A15 : GPP_A15 ==> USB_OC2# */
	PAD_CFG_NF(GPP_A15, NONE, DEEP, NF1),
	/* A16 : GPP_A16 ==> USB_OC3# */
	PAD_CFG_NF(GPP_A16, NONE, DEEP, NF1),
	/* A17 : GPP_A17 ==> NC */
	PAD_NC(GPP_A17, NONE),
	/* A18 : GPP_A18 ==> HDMI_HPD */
	PAD_CFG_NF(GPP_A18, NONE, DEEP, NF1),
	/* A19 : GPP_A19 ==> NC */
	PAD_NC(GPP_A19, NONE),
	/* A20 : GPP_A20 ==> NC */
	PAD_NC(GPP_A20, NONE),
	/* A21 : GPP_A21 ==> 3.3V_CAM_EN# */
	PAD_CFG_GPO(GPP_A21, 0, PLTRST),
	/* A22 : GPP_A22 ==> KB_DET# */
	PAD_CFG_GPI(GPP_A22, NONE, PLTRST),
	/* A23 : GPP_A23 ==> RECOVERY# */
	PAD_CFG_GPI(GPP_A23, NONE, DEEP),

	/* B0  : GPP_B0 ==> CORE_VID0 */
	PAD_CFG_NF(GPP_B0, NONE, DEEP, NF1),
	/* B1  : GPP_B1 ==> CORE_VID1 */
	PAD_CFG_NF(GPP_B1, NONE, DEEP, NF1),
	/* B2  : GPP_B2 ==> VRALERT_L */
	PAD_CFG_NF(GPP_B2, NONE, DEEP, NF1),
	/* B3  : GPP_B3 ==> TOUCH_SCREEN_PD# */
	PAD_CFG_GPO(GPP_B3, 0, PLTRST),
	/* B4  : GPP_B4 ==> TOUCH_SCREEN_DET# */
	PAD_CFG_GPI(GPP_B4, NONE, DEEP),
	/* B5  : GPP_B5 ==> ISH_I2C0_SDA */
	PAD_CFG_NF(GPP_B5, NONE, DEEP, NF1),
	/* B6  : GPP_B6 ==> ISH_I2C0_SCL */
	PAD_CFG_NF(GPP_B6, NONE, DEEP, NF1),
	/* B7  : GPP_B7 ==> NC */
	PAD_NC(GPP_B7, NONE),
	/* B8  : GPP_B8 ==> NC */
	PAD_NC(GPP_B8, NONE),
	/* B9  : GPP_B9 ==> NC */
	PAD_NC(GPP_B9, NONE),
	/* B10 : GPP_B10 ===> NC */
	PAD_NC(GPP_B10, NONE),
	/* B11 : GPP_B11 ==> TBT_I2C_INT# */
	PAD_CFG_GPI_APIC(GPP_B11, NONE, PLTRST, LEVEL, INVERT),
	/* B12 : GPP_B12 ==> SIO_SLP_S0# */
	PAD_CFG_NF(GPP_B12, NONE, DEEP, NF1),
	/* B13 : PLTRST# ==> PCH_PLTRST# */
	PAD_CFG_NF(GPP_B13, NONE, DEEP, NF1),
	/* B14 : GPP_B14 ==> SPKR (PIN STRAP, Top Swap Override) */
	PAD_CFG_NF(GPP_B14, NONE, DEEP, NF1),
	/* B15 : GPP_B15 ==> SPK_DET0# */
	PAD_CFG_GPI(GPP_B15, NONE, PLTRST),
	/* B16 : GPP_B16 ==> ONE_DIMM# */
	PAD_CFG_GPI(GPP_B16, NONE, PLTRST),
	/* B17 : GPP_B17 ==> HOST_SD_WP# */
	PAD_CFG_GPO(GPP_B17, 0, PLTRST),
	/* B18 : GPP_B18 ==> NRB_BIT (PIN STRAP, No Reboot) */
	PAD_NC(GPP_B18, NONE),
	/* B19 : GPP_B19 ==> D3_RST# */
	PAD_CFG_GPO(GPP_B19, 0, DEEP),
	/* B20 : GPP_B20 ==> LCD_CBL_DET# */
	PAD_CFG_GPI(GPP_B20, NONE, PLTRST),
	/* B21 : GPP_B21 ==> PCH_TOUCH_SCREEN_EN */
	PAD_CFG_GPO(GPP_B21, 0, DEEP),
	/* B22 : GPP_B22 ==> NC */
	PAD_NC(GPP_B22, NONE),
	/* B23 : GPP_B23 ==> NC (PIN STRAP, CPUNSSC frequency) */
	PAD_NC(GPP_B23, NONE),

	/* C0  : GPP_C0 ==> MEM_SMBCLK */
	PAD_CFG_NF(GPP_C0, NONE, DEEP, NF1),
	/* C1  : GPP_C1 ==> MEM_SMBDATA */
	PAD_CFG_NF(GPP_C1, NONE, DEEP, NF1),
	/* C2  : GPP_C2 ==> NC (PIN STRAP, TLS Confidentiality) */
	PAD_NC(GPP_C2, NONE),
	/* C3  : GPP_C3 ==> SML0_SMBCLK */
	PAD_CFG_NF(GPP_C3, NONE, DEEP, NF1),
	/* C4  : GPP_C4 ==> SML0_SMBDATA */
	PAD_CFG_NF(GPP_C4, NONE, DEEP, NF1),
	/* C5  : GPP_C5 ==> NC (PIN STRAP, Boot Strap 0) */
	PAD_NC(GPP_C5, NONE),
	/* C6  : GPP_C6 ==> SML1_SMBCLK */
	PAD_CFG_NF(GPP_C6, NONE, DEEP, NF1),
	/* C7  : GPP_C7 ==> SML1_SMBDATA */
	PAD_CFG_NF(GPP_C7, NONE, DEEP, NF1),
	/* C8  : GPP_C8 ==> WWAN_FULL_POWER_EN */
	PAD_CFG_GPO(GPP_C8, 1, DEEP),
	/* C9  : GPP_C9 ==> SBIOS_TX */
	PAD_CFG_GPO(GPP_C9, 0, PLTRST),
	/* C10 : GPP_C10 ==> NC */
	PAD_NC(GPP_C10, NONE),
	/* C11 : GPP_C11 ==> NC */
	PAD_NC(GPP_C11, NONE),
	/* C12 : GPP_C12 ==> NC */
	PAD_NC(GPP_C12, NONE),
	/* C13 : GPP_C13 ==> PCH_SSD_PWR_EN */
	PAD_CFG_GPO(GPP_C13, 1, DEEP),
	/* C14 : GPP_C14 ==> NC */
	PAD_NC(GPP_C14, NONE),
	/* C15 : GPP_C15 ==> NC */
	PAD_NC(GPP_C15, NONE),
	/* C16 : GPP_C16 ==> I2C0_SDA_TS */
	PAD_CFG_NF(GPP_C16, NONE, DEEP, NF1),
	/* C17 : GPP_C17 ==> I2C0_SCL_TS */
	PAD_CFG_NF(GPP_C17, NONE, DEEP, NF1),
	/* C18 : GPP_C18 ==> I2C1_SDA_TP */
	PAD_CFG_NF(GPP_C18, NONE, DEEP, NF1),
	/* C19 : GPP_C19 ==> I2C1_SCL_TP */
	PAD_CFG_NF(GPP_C19, NONE, DEEP, NF1),
	/* C20 : GPP_C20 ==> PCHRX_SERVOTX_UART */
	PAD_CFG_NF(GPP_C20, NONE, DEEP, NF1),
	/* C21 : CPP_G21 ==> PCHTX_SERVORX_UART */
	PAD_CFG_NF(GPP_C21, NONE, DEEP, NF1),
	/* C22 : GPP_C22 ==> H1_FLASH_WP */
	PAD_CFG_GPI(GPP_C22, NONE, DEEP),
	/* C23 : GPP_C23 ==> H1_PCH_INT# */
	PAD_CFG_GPI_APIC(GPP_C23, NONE, DEEP, LEVEL, INVERT),

	/* D0  : GPP_D0 ==> ISH_ACC1_INT */
	PAD_CFG_NF(GPP_D0, NONE, DEEP, NF1),
	/* D1  : GPP_D1 ==> ISH_ACC2_INT */
	PAD_CFG_NF(GPP_D1, NONE, DEEP, NF1),
	/* D2  : GPP_D2 ==> ISH_TABLE_MODE# */
	PAD_CFG_NF(GPP_D2, NONE, DEEP, NF1),
	/* D3  : GPP_D3 ==> ISH_ALS_INT# */
	PAD_CFG_NF(GPP_D3, NONE, DEEP, NF1),
	/* D4  : GPP_D4 ==> RT_FORCE_PWR */
	PAD_CFG_GPO(GPP_D4, 0, PLTRST),
	/* D5  : GPP_D5 ==> CLKREQ_PCIE#0 */
	PAD_CFG_NF(GPP_D5, NONE, DEEP, NF1),
	/* D6  : GPP_D6 ==> CLKREQ_PCIE#1 */
	PAD_CFG_NF(GPP_D6, NONE, DEEP, NF1),
	/* D7  : GPP_D7 ==> CLKREQ_PCIE#2 */
	PAD_CFG_NF(GPP_D7, NONE, DEEP, NF1),
	/* D8  : GPP_D8 ==> CLKREQ_PCIE#3 */
	PAD_CFG_NF(GPP_D8, NONE, DEEP, NF1),
	/* D9  : GPP_D9 ==> TBT_2_LSX_TX */
	PAD_CFG_NF(GPP_D9, NONE, DEEP, NF4),
	/* D10 : GPP_D10 ==> TBT_2_LSX_RX */
	PAD_CFG_NF(GPP_D10, NONE, DEEP, NF4),
	/* D11 : GPP_D11 ==> TBT_3_LSX_TX */
	PAD_CFG_NF(GPP_D11, NONE, DEEP, NF4),
	/* D12 : GPP_D12 ==> TBT_3_LSX_RX */
	PAD_CFG_NF(GPP_D12, DN_20K, DEEP, NF4),
	/* D13 : GPP_D13 ==> SML0B_SMLDATA */
	PAD_CFG_NF(GPP_D13, NONE, DEEP, NF2),
	/* D14 : GPP_D14 ==> SML0B_SMLCLK */
	PAD_CFG_NF(GPP_D14, NONE, DEEP, NF2),
	/* D15 : GPP_D15 ==> NC */
	PAD_NC(GPP_D15, NONE),
	/* D16 : GPP_D16 ==> SML0BALERT# */
	PAD_CFG_NF(GPP_D16, NONE, DEEP, NF2),
	/* D17 : GPP_D17 ==> ISH_NB_MODE# */
	PAD_CFG_NF(GPP_D17, NONE, DEEP, NF1),
	/* D18 : GPP_D18 ==> ISH_LID_CL#_NB */
	PAD_CFG_NF(GPP_D18, NONE, DEEP, NF1),
	/* D19 : GPP_D19 ==> NC */
	PAD_NC(GPP_D19, NONE),

	/* E0  : GPP_E0 ==> NC */
	PAD_NC(GPP_E0, NONE),
	/* E1  : GPP_E1 ==> TOUCH_SCREEN_INT# */
	PAD_CFG_GPI_APIC(GPP_E1, NONE, PLTRST, LEVEL, INVERT),
	/* E2  : GPP_E2 ==> MEDIACARD_IRQ# */
	PAD_CFG_GPI_APIC(GPP_E2, NONE, PLTRST, LEVEL, INVERT),
	/* E3  : GPP_E3 ==> MEM_INTERLEAVED */
	PAD_CFG_GPI(GPP_E3, NONE, PLTRST),
	/* E4  : GPP_E4 ==> NC */
	PAD_NC(GPP_E4, NONE),
	/* E5  : GPP_E5 ==> M2280_DEVSLP */
	PAD_CFG_NF(GPP_E5, NONE, DEEP, NF1),
	/* E6  : GPP_E6 ==> (PIN STRAP, Reserved) */
	PAD_NC(GPP_E6, NONE),
	/* E7  : CPU_GP1 ==> PCH_TOUCHPAD_INTR# */
	PAD_CFG_GPI_IRQ_WAKE(GPP_E7, NONE, PLTRST, LEVEL, INVERT),
	/* E8  : GPP_E8 ==> SECURE_BIO */
	PAD_CFG_GPO(GPP_E8, 0, PLTRST),
	/* E9  : GPP_E9 ==> OC0# */
	PAD_CFG_NF(GPP_E9, NONE, DEEP, NF1),
	/* E10 : GPP_E10 ==> HDMI_PD# */
	PAD_CFG_GPO(GPP_E10, 1, DEEP),
	/* E11 : GPP_E11 ==> VPRO_DET# */
	PAD_CFG_GPI(GPP_E11, NONE, PLTRST),
	/* E12 : GPP_E12 ==> RTC_DET# */
	PAD_CFG_GPI(GPP_E12, NONE, PLTRST),
	/* E13 : GPP_E13 ==> TBT_DET# */
	PAD_CFG_GPI(GPP_E13, NONE, DEEP),
	/* E14 : GPP_E14 ==> EPD_HPD */
	PAD_CFG_NF(GPP_E14, NONE, DEEP, NF1),
	/* E15 : GPP_E15 ==> ISH_LID_CL#_TAB */
	PAD_CFG_NF(GPP_E15, NONE, DEEP, NF1),
	/* E16 : GPP_E16 ==> NC */
	PAD_NC(GPP_E16, NONE),
	/* E17 : GPP_E17 ==> NC */
	PAD_NC(GPP_E17, NONE),
	/* E18 : GPP_E18 ==> TBT_LSX0_TXD */
	PAD_CFG_NF(GPP_E18, NONE, DEEP, NF4),
	/* E19 : GPP_E19 ==> TBT_LSX0_RXD */
	PAD_CFG_NF(GPP_E19, NONE, DEEP, NF4),
	/* E20 : GPP_E20 ==> TBT_LSX1_TXD */
	PAD_CFG_NF(GPP_E20, NONE, DEEP, NF4),
	/* E21 : GPP_E21 ==> TBT_LSX1_RXD */
	PAD_CFG_NF(GPP_E21, NONE, DEEP, NF4),
	/* E22 : GPP_E22 ==> NC */
	PAD_NC(GPP_E22, NONE),
	/* E23 : GPP_E23 ==> NC */
	PAD_NC(GPP_E23, NONE),

	/* F0  : GPP_F0 ==> BRI_DT_1P8 */
	PAD_CFG_NF(GPP_F0, NONE, DEEP, NF1),
	/* F1  : GPP_F1 ==> CNV_BRI_RSP_1P8 */
	PAD_CFG_NF(GPP_F1, NONE, DEEP, NF1),
	/* F2  : GPP_F2 ==> CNV_RGI_DT_1P8 */
	PAD_CFG_NF(GPP_F2, NONE, DEEP, NF1),
	/* F3  : GPP_F3 ==> CNV_RGI_RSP_1P8 */
	PAD_CFG_NF(GPP_F3, NONE, DEEP, NF1),
	/* F4  : GPP_F4 ==> NC */
	PAD_NC(GPP_F4, NONE),
	/* F5  : GPP_F5 ==> NC */
	PAD_NC(GPP_F5, NONE),
	/* F6  : GPP_F6 ==> NC */
	PAD_NC(GPP_F6, NONE),
	/* F7  : GPP_F7 ==> NC (PIN STRAP, Reserved) */
	PAD_NC(GPP_F7, NONE),
	/* F8  : GPP_F8 ==> NC */
	PAD_NC(GPP_F8, NONE),
	/* F9  : GPP_F9 ==> NC */
	PAD_NC(GPP_F9, NONE),
	/* F10 : GPP_F10 ==> NC (PIN STRAP, Reserved) */
	PAD_NC(GPP_F10, NONE),
	/* F11 : GPP_F11 ==> MEM_CONFIG0_1P8 */
	PAD_CFG_GPI(GPP_F11, NONE, DEEP),
	/* F12 : GPP_F12 ==> MEM_CONFIG1_1P8 */
	PAD_CFG_GPI(GPP_F12, NONE, DEEP),
	/* F13 : GPP_F13 ==> MEM_CONFIG2_1P8 */
	PAD_CFG_GPI(GPP_F13, NONE, DEEP),
	/* F14 : GPP_F14 ==> MEM_CONFIG3_1P8 */
	PAD_CFG_GPI(GPP_F14, NONE, DEEP),
	/* F15 : GPP_F15 ==> MEM_CONFIG4_1P8 */
	PAD_CFG_GPI(GPP_F15, NONE, DEEP),
	/* F16 : GPP_F16 ==> WWAN_BB_RST#_1P8 */
	PAD_CFG_GPO(GPP_F16, 1, DEEP),
	/* F17 : GPP_F17 ==> WWAN_GPIO_PERST# */
	PAD_CFG_GPO(GPP_F17, 0, DEEP),
	/* F18 : GPP_F18 ==> WWAN_GPIO_WAKE# */
	PAD_CFG_GPI_SCI_LOW(GPP_F18, NONE, DEEP, EDGE_SINGLE),
	/* F19 : GPP_F19 ==> CAM_MIC_CBL_DET# */
	PAD_CFG_GPI(GPP_F19, NONE, PLTRST),
	/* F20 : GPP_F20 ==> NC */
	PAD_NC(GPP_F20, NONE),
	/* F21 : GPP_F21 ==> NC */
	PAD_NC(GPP_F21, NONE),
	/* F22 : VNN_CTRL */
	PAD_CFG_NF(GPP_F22, NONE, DEEP, NF1),
	/* F23 : V1P05_CTRL */
	PAD_CFG_NF(GPP_F23, NONE, DEEP, NF1),

	/* H0  : GPPH0_BOOT_STRAP1 */
	PAD_NC(GPP_H0, NONE),
	/* H1  : GPPH1_BOOT_STRAP2 */
	PAD_NC(GPP_H1, NONE),
	/* H2  : GPPH2_BOOT_STRAP3 */
	PAD_NC(GPP_H2, NONE),
	/* H3  : GPP_H3 ==> NC */
	PAD_NC(GPP_H3, NONE),
	/* H4  : GPP_H4 ==> DDR_CHA_EN_1P8 */
	PAD_CFG_GPI(GPP_H4, NONE, DEEP),
	/* H5  : GPP_H5 ==> DDR_CHB_EN_1P8 */
	PAD_CFG_GPI(GPP_H5, NONE, DEEP),
	/* H6  : GPP_H6 ==> I2C_SDA_PCH_H1 */
	PAD_CFG_NF(GPP_H6, NONE, DEEP, NF1),
	/* H7  : GPP_H7 ==> I2C_SCL_PCH_H1 */
	PAD_CFG_NF(GPP_H7, NONE, DEEP, NF1),
	/* H8  : GPP_H8 ==> NC */
	PAD_NC(GPP_H8, NONE),
	/* H9  : GPP_H9 ==> NC */
	PAD_NC(GPP_H9, NONE),
	/* H10 : GPP_H10 ==> CLKREQ_PCIE#4 */
	PAD_CFG_NF(GPP_H10, NONE, DEEP, NF1),
	/* H11 : GPP_H11 ==> CLKREQ_PCIE#5 */
	PAD_CFG_NF(GPP_H11, NONE, DEEP, NF1),
	/* H12 : GPP_H12 ==> NC */
	PAD_NC(GPP_H12, NONE),
	/* H13 : GPP_H13 ==> NC */
	PAD_NC(GPP_H13, NONE),
	/* H14 : GPP_H14 ==> NC */
	PAD_NC(GPP_H14, NONE),
	/* H15 : GPP_H15 ==> NC */
	PAD_NC(GPP_H15, NONE),
	/* H16 : GPP_H16 ==> CPU_DPB_CTRL_CLK */
	PAD_CFG_NF(GPP_H16, NONE, DEEP, NF1),
	/* H17 : GPP_H17 ==> CPU_DPB_CTRL_DATA */
	PAD_CFG_NF(GPP_H17, NONE, DEEP, NF1),
	/* H18 : CPU_C10_GATE# ==> CPU_C10_GATE# */
	PAD_CFG_NF(GPP_H18, NONE, DEEP, NF1),
	/* H19 : GPP_H19 ==> NC */
	PAD_NC(GPP_H19, NONE),
	/* H20 : GPP_H20 ==> NC */
	PAD_NC(GPP_H20, NONE),
	/* H21 : GPP_H21 ==> NC */
	PAD_NC(GPP_H21, NONE),
	/* H22 : GPP_H22 ==> NC */
	PAD_NC(GPP_H22, NONE),
	/* H23 : GPP_H23 ==> NC */
	PAD_NC(GPP_H23, NONE),

	/* R0 : GPP_R0 ==> HDA_BCLK */
	PAD_CFG_NF(GPP_R0, NONE, DEEP, NF1),
	/* R1 : GPP_R1 ==> HDA_SYNC */
	PAD_CFG_NF(GPP_R1, NONE, DEEP, NF1),
	/* R2 : GPP_R2 ==> HDA_SDO (PIN STRAP, Flash Descriptor Security Override */
	PAD_CFG_NF(GPP_R2, NONE, DEEP, NF1),
	/* R3 : GPP_R3 ==> HDA_SDIO */
	PAD_CFG_NF(GPP_R3, NONE, DEEP, NF1),
	/* R4 : GPP_R4 ==> HDA_RST# */
	PAD_CFG_NF(GPP_R4, NONE, DEEP, NF1),
	/* R5 : GPP_R5 ==> NC */
	PAD_NC(GPP_R5, NONE),
	/* R6 : GPP_R6 ==> SD_PWR_EN1 */
	PAD_CFG_GPO(GPP_R6, 0, PLTRST),
	/* R7 : GPP_R7 ==> SD_PWR_EN2 */
	PAD_CFG_GPO(GPP_R7, 0, PLTRST),

	/* S0 : GPP_S0 ==> NC */
	PAD_NC(GPP_S0, NONE),
	/* S1 : GPP_S1 ==> NC */
	PAD_NC(GPP_S1, NONE),
	/* S2 : GPP_S2 ==> NC */
	PAD_NC(GPP_S2, NONE),
	/* S3 : GPP_S3 ==> NC */
	PAD_NC(GPP_S3, NONE),
	/* S4 : GPP_S4 ==> NC */
	PAD_NC(GPP_S4, NONE),
	/* S5 : GPP_S5 ==> NC */
	PAD_NC(GPP_S5, NONE),
	/* S6 : GPP_S6 ==> NC */
	PAD_NC(GPP_S6, NONE),
	/* S7 : GPP_S7 ==> NC */
	PAD_NC(GPP_S7, NONE),

	/* GPD0: GPD0 ==> PCH_BATLOW# */
	PAD_CFG_NF(GPD0, NONE, DEEP, NF1),
	/* GPD1: GPD1 ==> AC_PRESENT */
	PAD_CFG_NF(GPD1, NONE, DEEP, NF1),
	/* GPD2: GPD2 ==> LAN_WAKE# */
	PAD_CFG_NF(GPD2, NONE, DEEP, NF1),
	/* GPD3: GPD3 ==> SIO_PWRBTN# */
	PAD_CFG_NF(GPD3, UP_20K, DEEP, NF1),
	/* GPD4: GPD4 ==> SIO_SLP_S3# */
	PAD_CFG_NF(GPD4, NONE, DEEP, NF1),
	/* GPD5: GPD5 ==> SIO_SLP_S4# */
	PAD_CFG_NF(GPD5, NONE, DEEP, NF1),
	/* GPD6: GPD6 ==> SIO_SLP_A# */
	PAD_CFG_NF(GPD6, NONE, DEEP, NF1),
	/* GPD7: GPD7 ==> PCH_TBT_PERST# (PIN STRAP, Reserved) */
	PAD_CFG_GPO(GPD7, 0, PLTRST),
	/* GPD8: GPD8 ==> SUSCLK */
	PAD_CFG_NF(GPD8, NONE, DEEP, NF1),
	/* GPD9: GPD9 ==> SIO_SLP_WLAN# */
	PAD_CFG_NF(GPD9, NONE, DEEP, NF1),
	/* GPD10: GPD10 ==> SIO_SLP_S5# */
	PAD_CFG_NF(GPD10, NONE, DEEP, NF1),
	/* GPD11: GPD11 ==> PM_LANPHY_EN */
	PAD_CFG_NF(GPD11, NONE, DEEP, NF1),
};

const struct pad_config *__weak variant_base_gpio_table(size_t *num)
{
	*num = ARRAY_SIZE(gpio_table);
	return gpio_table;
}

static const struct cros_gpio cros_gpios[] = {
	CROS_GPIO_REC_AL(GPIO_REC_MODE, CROS_GPIO_NAME),
	CROS_GPIO_WP_AH(GPIO_PCH_WP, CROS_GPIO_NAME),
};

const struct cros_gpio *__weak variant_cros_gpios(size_t *num)
{
	*num = ARRAY_SIZE(cros_gpios);
	return cros_gpios;
}

/* Weak implementation of overrides */
const struct pad_config *__weak variant_override_gpio_table(size_t *num)
{
	*num = 0;
	return NULL;
}

/* Weak implementation of early gpio */
const struct pad_config *__weak variant_early_gpio_table(size_t *num)
{
	*num = 0;
	return NULL;
}

int __weak has_360_sensor_board(void)
{
	return 0;
}
