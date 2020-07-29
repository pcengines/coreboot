/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __SOC_NVIDIA_TEGRA210_PINMUX_H__
#define __SOC_NVIDIA_TEGRA210_PINMUX_H__

#include <soc/nvidia/tegra/gpio.h>
#include <soc/nvidia/tegra/pinmux.h>

enum {
	PINMUX_FUNC_MASK = 3 << 0,

	PINMUX_PULL_MASK = 3 << 2,
	PINMUX_PULL_NONE = 0 << 2,
	PINMUX_PULL_DOWN = 1 << 2,
	PINMUX_PULL_UP = 2 << 2,

	PINMUX_TRISTATE = 1 << 4,
	PINMUX_PARKED = 1 << 5,
	PINMUX_INPUT_ENABLE = 1 << 6,
	PINMUX_LOCK = 1 << 7,
	PINMUX_LPDR = 1 << 8,
	PINMUX_HSM = 1 << 9,
	PINMUX_IO_HV = 1 << 10,
	PINMUX_OPEN_DRAIN = 1 << 11,
	PINMUX_SCHMT = 1 << 12,

	PINMUX_DRIVE_1X = 0 << 13,
	PINMUX_DRIVE_2X = 1 << 13,
	PINMUX_DRIVE_3X = 2 << 13,
	PINMUX_DRIVE_4X = 3 << 13,
};

/* GPIO index constants. */

#define GPIO_PORT_CONSTANTS(port) \
	GPIO_##port##0_INDEX, GPIO_##port##1_INDEX, GPIO_##port##2_INDEX, \
	GPIO_##port##3_INDEX, GPIO_##port##4_INDEX, GPIO_##port##5_INDEX, \
	GPIO_##port##6_INDEX, GPIO_##port##7_INDEX

enum {
	GPIO_PORT_CONSTANTS(A),
	GPIO_PORT_CONSTANTS(B),
	GPIO_PORT_CONSTANTS(C),
	GPIO_PORT_CONSTANTS(D),
	GPIO_PORT_CONSTANTS(E),
	GPIO_PORT_CONSTANTS(F),
	GPIO_PORT_CONSTANTS(G),
	GPIO_PORT_CONSTANTS(H),
	GPIO_PORT_CONSTANTS(I),
	GPIO_PORT_CONSTANTS(J),
	GPIO_PORT_CONSTANTS(K),
	GPIO_PORT_CONSTANTS(L),
	GPIO_PORT_CONSTANTS(M),
	GPIO_PORT_CONSTANTS(N),
	GPIO_PORT_CONSTANTS(O),
	GPIO_PORT_CONSTANTS(P),
	GPIO_PORT_CONSTANTS(Q),
	GPIO_PORT_CONSTANTS(R),
	GPIO_PORT_CONSTANTS(S),
	GPIO_PORT_CONSTANTS(T),
	GPIO_PORT_CONSTANTS(U),
	GPIO_PORT_CONSTANTS(V),
	GPIO_PORT_CONSTANTS(W),
	GPIO_PORT_CONSTANTS(X),
	GPIO_PORT_CONSTANTS(Y),
	GPIO_PORT_CONSTANTS(Z),
	GPIO_PORT_CONSTANTS(AA),
	GPIO_PORT_CONSTANTS(BB),
	GPIO_PORT_CONSTANTS(CC),
	GPIO_PORT_CONSTANTS(DD),
	GPIO_PORT_CONSTANTS(EE),
	GPIO_PORT_CONSTANTS(FF),
	GPIO_NONE_INDEX = 0,
};

#define PINMUX_CONSTANTS_GPIO(name, gpio) \
	PINMUX_GPIO_##gpio = PINMUX_##name##_INDEX

#define PINMUX_CONSTANTS(index, name, por_pu, gpio, has_gpio, \
				func0, func1, func2, func3) \
	PINMUX_##name##_INDEX = index, \
	PINMUX_##name##_FUNC_##func0 = 0, \
	PINMUX_##name##_FUNC_##func1 = 1, \
	PINMUX_##name##_FUNC_##func2 = 2, \
	PINMUX_##name##_FUNC_##func3 = 3, \
	PAD_TO_GPIO_##name = GPIO_##gpio##_INDEX, \
	PAD_HAS_GPIO_##name = has_gpio, \
	PAD_POR_PU_##name = por_pu

#define PAD_GPIO(index, name, por_pu, gpio, func0, func1, func2, func3) \
	PINMUX_CONSTANTS(index, name, por_pu, gpio, 1, \
				func0, func1, func2, func3), \
	PINMUX_CONSTANTS_GPIO(name, gpio)

#define PAD_NO_GPIO(index, name, por_pu, func0, func1, func2, func3) \
	PINMUX_CONSTANTS(index, name, por_pu, NONE, 0, \
				func0, func1, func2, func3)

enum {
	/* Power-on-reset pull states. */
	POR_PU = 2,
	POR_PD = 1,
	POR_NP = 0,

	PAD_GPIO(0, SDMMC1_CLK,  POR_PD, M0, SDMMC1, RES1, RES2, RES3),
	PAD_GPIO(1, SDMMC1_CMD,  POR_PU, M1, SDMMC1, RES1, RES2, RES3),
	PAD_GPIO(2, SDMMC1_DAT3, POR_PU, M2, SDMMC1, RES1, RES2, RES3),
	PAD_GPIO(3, SDMMC1_DAT2, POR_PU, M3, SDMMC1, RES1, RES2, RES3),
	PAD_GPIO(4, SDMMC1_DAT1, POR_PU, M4, SDMMC1, RES1, RES2, RES3),
	PAD_GPIO(5, SDMMC1_DAT0, POR_PU, M5, SDMMC1, RES1, RES2, RES3),
	PAD_GPIO(6, SDMMC3_CLK,  POR_PD, P0, SDMMC3, RES1, RES2, RES3),
	PAD_GPIO(7, SDMMC3_CMD,  POR_PU, P1, SDMMC3, RES1, RES2, RES3),
	PAD_GPIO(8, SDMMC3_DAT0, POR_PU, P2, SDMMC3, RES1, RES2, RES3),
	PAD_GPIO(9, SDMMC3_DAT1, POR_PU, P3, SDMMC3, RES1, RES2, RES3),
	PAD_GPIO(10, SDMMC3_DAT2, POR_PU, P4, SDMMC3, RES1, RES2, RES3),
	PAD_GPIO(11, SDMMC3_DAT3, POR_PU, P5, SDMMC3, RES1, RES2, RES3),
	/* GPIO12 - unused */
	/* GPIO13 - unused */
	PAD_GPIO(14, PEX_L0_RST_N, POR_NP, A0, PE0, RES1, RES2, RES3),
	PAD_GPIO(15, PEX_L0_CLKREQ_N, POR_NP, A1, PE0, RES1, RES2, RES3),
	PAD_GPIO(16, PEX_WAKE_N, POR_NP, A2, PE, RES1, RES2, RES3),
	PAD_GPIO(17, PEX_L1_RST_N, POR_NP, A3, PE1, RES1, RES2, RES3),
	PAD_GPIO(18, PEX_L1_CLKREQ_N, POR_NP, A4, PE1, RES1, RES2, RES3),
	PAD_GPIO(19, SATA_LED_ACTIVE, POR_NP, A5, SATA, RES1, RES2, RES3),
	PAD_GPIO(20, SPI1_MOSI, POR_PD, C0, SPI1, RES1, RES2, RES3),
	PAD_GPIO(21, SPI1_MISO, POR_PD, C1, SPI1, RES1, RES2, RES3),
	PAD_GPIO(22, SPI1_SCK, POR_PD, C2, SPI1, RES1, RES2, RES3),
	PAD_GPIO(23, SPI1_CS0, POR_PU, C3, SPI1, RES1, RES2, RES3),
	PAD_GPIO(24, SPI1_CS1, POR_PU, C4, SPI1, RES1, RES2, RES3),
	PAD_GPIO(25, SPI2_MOSI, POR_PD, B4, SPI2, DTV, RES2, RES3),
	PAD_GPIO(26, SPI2_MISO, POR_PD, B5, SPI2, DTV, RES2, RES3),
	PAD_GPIO(27, SPI2_SCK, POR_PD, B6, SPI2, DTV, RES2, RES3),
	PAD_GPIO(28, SPI2_CS0, POR_PU, B7, SPI2, DTV, RES2, RES3),
	PAD_GPIO(29, SPI2_CS1, POR_PU, DD0, SPI2, RES1, RES2, RES3),
	PAD_GPIO(30, SPI4_MOSI, POR_PD, C7, SPI4, RES1, RES2, RES3),
	PAD_GPIO(31, SPI4_MISO, POR_PD, D0, SPI4, RES1, RES2, RES3),
	PAD_GPIO(32, SPI4_SCK, POR_PD, C5, SPI4, RES1, RES2, RES3),
	PAD_GPIO(33, SPI4_CS0, POR_PU, C6, SPI4, RES1, RES2, RES3),
	PAD_GPIO(34, QSPI_SCK, POR_PU, EE0, QSPI, RES1, RES2, RES3),
	PAD_GPIO(35, QSPI_CS_N, POR_PU, EE1, QSPI, RES1, RES2, RES3),
	PAD_GPIO(36, QSPI_IO0, POR_PU, EE2, QSPI, RES1, RES2, RES3),
	PAD_GPIO(37, QSPI_IO1, POR_PU, EE3, QSPI, RES1, RES2, RES3),
	PAD_GPIO(38, QSPI_IO2, POR_PU, EE4, QSPI, RES1, RES2, RES3),
	PAD_GPIO(39, QSPI_IO3, POR_PU, EE5, QSPI, RES1, RES2, RES3),
	/* GPIO40 - unused */
	PAD_GPIO(41, DMIC1_CLK, POR_PD, E0, DMIC1, I2S3, RES2, RES3),
	PAD_GPIO(42, DMIC1_DAT, POR_PD, E1, DMIC1, I2S3, RES2, RES3),
	PAD_GPIO(43, DMIC2_CLK, POR_PD, E2, DMIC2, I2S3, RES2, RES3),
	PAD_GPIO(44, DMIC2_DAT, POR_PD, E3, DMIC2, I2S3, RES2, RES3),
	PAD_GPIO(45, DMIC3_CLK, POR_PD, E4, DMIC3, I2S5A, RES2, RES3),
	PAD_GPIO(46, DMIC3_DAT, POR_PD, E5, DMIC3, I2S5A, RES2, RES3),
	PAD_GPIO(47, GEN1_I2C_SDA, POR_NP, J0, I2C1, RES1, RES2, RES3),
	PAD_GPIO(48, GEN1_I2C_SCL, POR_NP, J1, I2C1, RES1, RES2, RES3),
	PAD_GPIO(49, GEN2_I2C_SCL, POR_NP, J2, I2C2, RES1, RES2, RES3),
	PAD_GPIO(50, GEN2_I2C_SDA, POR_NP, J3, I2C2, RES1, RES2, RES3),
	PAD_GPIO(51, GEN3_I2C_SCL, POR_NP, F0, I2C3, RES1, RES2, RES3),
	PAD_GPIO(52, GEN3_I2C_SDA, POR_NP, F1, I2C3, RES1, RES2, RES3),
	PAD_GPIO(53, CAM_I2C_SCL, POR_NP, S2, I2C3, I2CVI, RES2, RES3),
	PAD_GPIO(54, CAM_I2C_SDA, POR_NP, S3, I2C3, I2CVI, RES2, RES3),
	PAD_GPIO(55, PWR_I2C_SCL, POR_NP, Y3, I2CPMU, RES1, RES2, RES3),
	PAD_GPIO(56, PWR_I2C_SDA, POR_NP, Y4, I2CPMU, RES1, RES2, RES3),
	PAD_GPIO(57, UART1_TX, POR_PD, U0, UARTA, RES1, RES2, RES3),
	PAD_GPIO(58, UART1_RX, POR_PD, U1, UARTA, RES1, RES2, RES3),
	PAD_GPIO(59, UART1_RTS, POR_PD, U2, UARTA, RES1, RES2, RES3),
	PAD_GPIO(60, UART1_CTS, POR_PD, U3, UARTA, RES1, RES2, RES3),
	PAD_GPIO(61, UART2_TX, POR_PD, G0, UARTB, I2S4A, SPDIF, UART),
	PAD_GPIO(62, UART2_RX, POR_PD, G1, UARTB, I2S4A, SPDIF, UART),
	PAD_GPIO(63, UART2_RTS, POR_PD, G2, UARTB, I2S4A, RES2, UART),
	PAD_GPIO(64, UART2_CTS, POR_PD, G3, UARTB, I2S4A, RES2, UART),
	PAD_GPIO(65, UART3_TX, POR_PD, D1, UARTC, SPI4, RES2, RES3),
	PAD_GPIO(66, UART3_RX, POR_PD, D2, UARTC, SPI4, RES2, RES3),
	PAD_GPIO(67, UART3_RTS, POR_PD, D3, UARTC, SPI4, RES2, RES3),
	PAD_GPIO(68, UART3_CTS, POR_PD, D4, UARTC, SPI4, RES2, RES3),
	PAD_GPIO(69, UART4_TX, POR_PD, I4, UARTD, UART, RES2, RES3),
	PAD_GPIO(70, UART4_RX, POR_PD, I5, UARTD, UART, RES2, RES3),
	PAD_GPIO(71, UART4_RTS, POR_PD, I6, UARTD, UART, RES2, RES3),
	PAD_GPIO(72, UART4_CTS, POR_PD, I7, UARTD, UART, RES2, RES3),
	PAD_GPIO(73, DAP1_FS, POR_PD, B0, I2S1, RES1, RES2, RES3),
	PAD_GPIO(74, DAP1_DIN, POR_PD, B1, I2S1, RES1, RES2, RES3),
	PAD_GPIO(75, DAP1_DOUT, POR_PD, B2, I2S1, RES1, RES2, RES3),
	PAD_GPIO(76, DAP1_SCLK, POR_PD, B3, I2S1, RES1, RES2, RES3),
	PAD_GPIO(77, DAP2_FS, POR_PD, AA0, I2S2, RES1, RES2, RES3),
	PAD_GPIO(78, DAP2_DIN, POR_PD, AA1, I2S2, RES1, RES2, RES3),
	PAD_GPIO(79, DAP2_DOUT, POR_PD, AA2, I2S2, RES1, RES2, RES3),
	PAD_GPIO(80, DAP2_SCLK, POR_PD, AA3, I2S2, RES1, RES2, RES3),
	PAD_GPIO(81, DAP4_FS, POR_PD, J4, I2S4B, RES1, RES2, RES3),
	PAD_GPIO(82, DAP4_DIN, POR_PD, J5, I2S4B, RES1, RES2, RES3),
	PAD_GPIO(83, DAP4_DOUT, POR_PD, J6, I2S4B, RES1, RES2, RES3),
	PAD_GPIO(84, DAP4_SCLK, POR_PD, J7, I2S4B, RES1, RES2, RES3),
	PAD_GPIO(85, CAM1_MCLK, POR_PD, S0, EXTPERIPH3, RES1, RES2, RES3),
	PAD_GPIO(86, CAM2_MCLK, POR_PD, S1, EXTPERIPH3, RES1, RES2, RES3),
	PAD_NO_GPIO(87, JTAG_RTCK, POR_PU, JTAG, RES1, RES2, RES3),
	PAD_NO_GPIO(88, CLK_32K_IN, POR_NP, CLK_32K_IN, RES1, RES2, RES3),
	PAD_GPIO(89, CLK_32K_OUT, POR_PD, Y5, SOC, BLINK, RES2, RES3),
	PAD_NO_GPIO(90, BATT_BCL, POR_NP, BCL, RES1, RES2, RES3),
	PAD_NO_GPIO(91, CLK_REQ, POR_NP, CLK_REQ, RES1, RES2, RES3),
	PAD_NO_GPIO(92, CPU_PWR_REQ, POR_NP, CPU, RES1, RES2, RES3),
	PAD_NO_GPIO(93, PWR_INT_N, POR_NP, PMI, RES1, RES2, RES3),
	PAD_NO_GPIO(94, SHUTDOWN, POR_NP, SHUTDOWN, RES1, RES2, RES3),
	PAD_NO_GPIO(95, CORE_PWR_REQ, POR_NP, PWRON, RES1, RES2, RES3),
	PAD_GPIO(96, AUD_MCLK, POR_PD, BB0, AUD, RES1, RES2, RES3),
	PAD_GPIO(97, DVFS_PWM, POR_PD, BB1, RES0, CLDVFS, SPI3, RES3),
	PAD_GPIO(98, DVFS_CLK, POR_PU, BB2, RES0, CLDVFS, SPI3, RES3),
	PAD_GPIO(99, GPIO_X1_AUD, POR_PD, BB3, RES0, RES1, SPI3, RES3),
	PAD_GPIO(100, GPIO_X3_AUD, POR_PU, BB4, RES0, RES1, SPI3, RES3),
	PAD_NO_GPIO(101, GPIO_PCC7, POR_NP, RES0, RES1, RES2, RES3),
	PAD_GPIO(102, HDMI_CEC, POR_NP, CC0, CEC, RES1, RES2, RES3),
	PAD_GPIO(103, HDMI_INT_DP_HPD, POR_PD, CC1, DP, RES1, RES2, RES3),
	PAD_GPIO(104, SPDIF_OUT, POR_PU, CC2, SPDIF, RES1, RES2, I2C3),
	PAD_GPIO(105, SPDIF_IN, POR_PD, CC3, SPDIF, RES1, RES2, I2C3),
	PAD_GPIO(106, USB_VBUS_EN0, POR_NP, CC4, USB, RES1, RES2, RES3),
	PAD_GPIO(107, USB_VBUS_EN1, POR_NP, CC5, USB, RES1, RES2, RES3),
	PAD_GPIO(108, DP_HPD0, POR_PD, CC6, DP, RES1, RES2, RES3),
	PAD_GPIO(109, WIFI_EN, POR_PD, H0, RES0, RES1, RES2, RES3),
	PAD_GPIO(110, WIFI_RST, POR_PD, H1, RES0, RES1, RES2, RES3),
	PAD_GPIO(111, WIFI_WAKE_AP, POR_PD, H2, RES0, RES1, RES2, RES3),
	PAD_GPIO(112, AP_WAKE_BT, POR_PD, H3, RES0, UARTB, SPDIF, RES3),
	PAD_GPIO(113, BT_RST, POR_PD, H4, RES0, UARTB, SPDIF, RES3),
	PAD_GPIO(114, BT_WAKE_AP, POR_PD, H5, RES0, RES1, RES2, RES3),
	PAD_GPIO(115, AP_WAKE_NFC, POR_PD, H7, RES0, RES1, RES2, RES3),
	PAD_GPIO(116, NFC_EN, POR_PD, I0, RES0, RES1, RES2, RES3),
	PAD_GPIO(117, NFC_INT, POR_PD, I1, RES0, RES1, RES2, RES3),
	PAD_GPIO(118, GPS_EN, POR_PD, I2, RES0, RES1, RES2, RES3),
	PAD_GPIO(119, GPS_RST, POR_PD, I3, RES0, RES1, RES2, RES3),
	PAD_GPIO(120, CAM_RST, POR_PD, S4, VGP1, RES1, RES2, RES3),
	PAD_GPIO(121, CAM_AF_EN, POR_PD, S5, VIMCLK, VGP2, RES2, RES3),
	PAD_GPIO(122, CAM_FLASH_EN, POR_PD, S6, VIMCLK, VGP3, RES2, RES3),
	PAD_GPIO(123, CAM1_PWDN, POR_PD, S7, VGP4, RES1, RES2, RES3),
	PAD_GPIO(124, CAM2_PWDN, POR_PD, T0, VGP5, RES1, RES2, RES3),
	PAD_GPIO(125, CAM1_STROBE, POR_PD, T1, VGP6, RES1, RES2, RES3),
	PAD_GPIO(126, LCD_TE, POR_PD, Y2, DISPLAYA, RES1, RES2, RES3),
	PAD_GPIO(127, LCD_BL_PWM, POR_PD, V0, DISPLAYA, PWM0, SOR0, RES3),
	PAD_GPIO(128, LCD_BL_EN, POR_PD, V1, RES0, RES1, RES2, RES3),
	PAD_GPIO(129, LCD_RST, POR_PD, V2, RES0, RES1, RES2, RES3),
	PAD_GPIO(130, LCD_GPIO1, POR_PD, V3, DISPLAYB, RES1, RES2, RES3),
	PAD_GPIO(131, LCD_GPIO2, POR_PD, V4, DISPLAYB, PWM1, RES2, SOR1),
	PAD_GPIO(132, AP_READY, POR_PD, V5, RES0, RES1, RES2, RES3),
	PAD_GPIO(133, TOUCH_RST, POR_PD, V6, RES0, RES1, RES2, RES3),
	PAD_GPIO(134, TOUCH_CLK, POR_PD, V7, TOUCH, RES1, RES2, RES3),
	PAD_GPIO(135, MODEM_WAKE_AP, POR_PD, X0, RES0, RES1, RES2, RES3),
	PAD_GPIO(136, TOUCH_INT, POR_PD, X1, RES0, RES1, RES2, RES3),
	PAD_GPIO(137, MOTION_INT, POR_PD, X2, RES0, RES1, RES2, RES3),
	PAD_GPIO(138, ALS_PROX_INT, POR_PD, X3, RES0, RES1, RES2, RES3),
	PAD_GPIO(139, TEMP_ALERT, POR_PD, X4, RES0, RES1, RES2, RES3),
	PAD_GPIO(140, BUTTON_POWER_ON, POR_PU, X5, RES0, RES1, RES2, RES3),
	PAD_GPIO(141, BUTTON_VOL_UP, POR_PU, X6, RES0, RES1, RES2, RES3),
	PAD_GPIO(142, BUTTON_VOL_DOWN, POR_PU, X7, RES0, RES1, RES2, RES3),
	PAD_GPIO(143, BUTTON_SLIDE_SW, POR_PU, Y0, RES0, RES1, RES2, RES3),
	PAD_GPIO(144, BUTTON_HOME, POR_PU, Y1, RES0, RES1, RES2, RES3),
	PAD_NO_GPIO(145, GPIO_PA6, POR_NP, SATA, RES1, RES2, RES3),
	PAD_NO_GPIO(146, GPIO_PE6, POR_PD, RES0, I2S5A, PWM2, RES3),
	PAD_NO_GPIO(147, GPIO_PE7, POR_PD, RES0, I2S5A, PWM3, RES3),
	PAD_NO_GPIO(148, GPIO_PH6, POR_PD, RES0, RES1, RES2, RES3),
	PAD_GPIO(149, GPIO_PK0, POR_PD, K0, IQC0, I2S5B, RES2, RES3),
	PAD_GPIO(150, GPIO_PK1, POR_PD, K1, IQC0, I2S5B, RES2, RES3),
	PAD_GPIO(151, GPIO_PK2, POR_PD, K2, IQC0, I2S5B, RES2, RES3),
	PAD_NO_GPIO(152, GPIO_PK3, POR_PD, IQC0, I2S5B, RES2, RES3),
	PAD_NO_GPIO(153, GPIO_PK4, POR_PD, IQC1, RES1, RES2, RES3),
	PAD_NO_GPIO(154, GPIO_PK5, POR_PD, IQC1, RES1, RES2, RES3),
	PAD_NO_GPIO(155, GPIO_PK6, POR_PD, IQC1, RES1, RES2, RES3),
	PAD_NO_GPIO(156, GPIO_PK7, POR_PD, IQC1, RES1, RES2, RES3),
	PAD_NO_GPIO(157, GPIO_PL0, POR_PD, RES0, RES1, RES2, RES3),
	PAD_NO_GPIO(158, GPIO_PL1, POR_PD, SOC, RES1, RES2, RES3),
	PAD_NO_GPIO(159, GPIO_PZ0, POR_PD, VIMCLK2, RES1, RES2, RES3),
	PAD_GPIO(160, GPIO_PZ1, POR_PD, Z1, VIMCLK2, SDMMC1, RES2, RES3),
	PAD_NO_GPIO(161, GPIO_PZ2, POR_PD, SDMMC3, CCLA, RES2, RES3),
	PAD_NO_GPIO(162, GPIO_PZ3, POR_PD, SDMMC3, RES1, RES2, RES3),
	PAD_GPIO(163, GPIO_PZ4, POR_PD, Z4, SDMMC1, RES1, RES2, RES3),
	PAD_NO_GPIO(164, GPIO_PZ5, POR_PD, SOC, RES1, RES2, RES3),
};

#endif	/* __SOC_NVIDIA_TEGRA210_PINMUX_H__ */
