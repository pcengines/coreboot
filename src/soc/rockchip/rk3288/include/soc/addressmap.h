/* SPDX-License-Identifier: GPL-2.0-only */
/* This file is part of the coreboot project. */

#ifndef __SOC_ROCKCHIP_RK3288_ADDRESSMAP_H__
#define __SOC_ROCKCHIP_RK3288_ADDRESSMAP_H__

#define MAX_DRAM_ADDRESS	0xFE000000

#define SDMMC1_BASE		0xFF0C0000
#define SDMMC0_BASE		0xFF0D0000
#define EMMC_BASE		0xFF0F0000
#define SARADC_BASE		0xFF100000

#define SPI0_BASE		0xFF110000
#define SPI1_BASE		0xFF120000
#define SPI2_BASE		0xFF130000

#define I2C1_BASE		0xFF140000
#define I2C3_BASE		0xFF150000
#define I2C4_BASE		0xFF160000
#define I2C5_BASE		0xFF170000
#define UART0_BASE		0xFF180000
#define UART1_BASE		0xFF190000
#define DMAC_PERI_BASE		0xFF250000
#define TSADC_BASE		0xFF280000

#define NANDC0_BASE		0xFF400000
#define NANDC1_BASE		0xFF410000

#define USB_HOST0_EHCI_BASE	0xFF500000
#define USB_HOST0_OHCI_BASE	0xFF520000
#define USB_HOST1_BASE		0xFF540000
#define USB_OTG_BASE		0xFF580000

#define DMAC_BUS_BASE		0xFF600000

#define DDR_PCTL0_BASE		0xFF610000
#define DDR_PCTL1_BASE		0xFF630000
#define DDR_PUBL0_BASE		0xFF620000
#define DDR_PUBL1_BASE		0xFF640000

#define I2C0_BASE		0xFF650000
#define I2C2_BASE		0xFF660000
#define DW_PWM0123_BASE		0xFF670000
#define RK_PWM_BASE		0xFF680000
#define UART2_BASE		0xFF690000
#define TIMER0_BASE		0xFF6B0000

#define SRAM_BASE		0xFF700000
#define PMU_BASE		0xFF730000
#define GRF_SECURE_BASE		0xFF740000
#define GPIO0_BASE		0xFF750000
#define CRU_BASE		0xFF760000
#define GRF_BASE		0xFF770000
#define GPIO1_BASE		0xFF780000
#define GPIO2_BASE		0xFF790000
#define GPIO3_BASE		0xFF7A0000
#define GPIO4_BASE		0xFF7B0000
#define GPIO5_BASE		0xFF7C0000
#define GPIO6_BASE		0xFF7D0000
#define GPIO7_BASE		0xFF7E0000
#define GPIO8_BASE		0xFF7F0000

#define TIMER6_BASE		0xFF810000
#define TIMER7_BASE		0xFF810020

#define CRYPTO_BASE		0xFF8A0000

#define VOP_BIG_BASE		0xFF930000
#define VOP_LIT_BASE		0xFF940000
#define EDP_BASE		0xFF970000

#define HDMI_TX_BASE		0xFF980000

#define	SERVICE_CORE_BASE	0xFFA80000
#define	SERVICE_DMA_BASE	0xFFA90000
#define	SERVICE_GPU_BASE	0xFFAA0000
#define	SERVICE_PERI_BASE	0xFFAB0000
#define	SERVICE_BUS_BASE	0xFFAC0000
#define	SERVICE_VIO_BASE	0xFFAD0000
#define	SERVICE_VPU_BASE	0xFFAE0000
#define	SERVICE_HEVC_BASE	0xFFAF0000

#define EFUSE_BASE		0xFFB40000

#define CORE_GICD_BASE		0xFFC01000
#define CORE_GICC_BASE		0xFFC02000
#define CPU_AXI_BUS_BASE	0xFFE00000

#define BOOT_ROM_BASE		0xFFFF0000
#define BOOT_ROM_CHIP_VER	(BOOT_ROM+0x27F0)
#define IC_BASES		{ I2C0_BASE, I2C1_BASE, I2C2_BASE, \
			I2C3_BASE, I2C4_BASE, I2C5_BASE }

#endif	/* __SOC_ROCKCHIP_RK3288_ADDRESSMAP_H__ */
