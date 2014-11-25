/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <t66/mx6_util.h>
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

iomux_v3_cfg_t const gpio_pads[] = {
	/* project detect */
	MX6_PAD_NANDF_D1__GPIO2_IO01		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D2__GPIO2_IO02		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D3__GPIO2_IO03		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* PCB detect */
	MX6_PAD_NANDF_D4__GPIO2_IO04		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D5__GPIO2_IO05		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* orange power led, t66 EVT */
	MX6_PAD_GPIO_1__GPIO1_IO01			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* orange power led, t66 DVT & PVT */
	MX6_PAD_EIM_D22__GPIO3_IO22			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* blue power led */
	MX6_PAD_GPIO_18__GPIO7_IO13			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* AUTO SDN, t66 EVT */
	MX6_PAD_EIM_D31__GPIO3_IO31			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* AUTO SDN, t66 DVT & PVT */
	MX6_PAD_KEY_ROW4__GPIO4_IO15		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* etnernet reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pcie reset */
	MX6_PAD_GPIO_19__GPIO4_IO05			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* PWR_BTN_SNS */
	MX6_PAD_EIM_CS0__GPIO2_IO23			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* soft off */
	MX6_PAD_KEY_COL4__GPIO4_IO14		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* power button detect */
	MX6_PAD_GPIO_17__GPIO7_IO12			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* last state detect */
	MX6_PAD_GPIO_8__GPIO1_IO08			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* audio reset */
	MX6_PAD_KEY_COL2__GPIO4_IO10		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* eMMC reset */
	MX6_PAD_NANDF_CS0__GPIO6_IO11		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* eMMC write protect indicator*/
	MX6_PAD_EIM_D26__GPIO3_IO26			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* 2.5V power enable */
	MX6_PAD_EIM_D21__GPIO3_IO21			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* USB_PWR1 */
	MX6_PAD_NANDF_WP_B__GPIO6_IO09		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* USB_PWR2 */
	MX6_PAD_NANDF_CS1__GPIO6_IO14		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* USB_PWR3 */
	MX6_PAD_GPIO_5__GPIO1_IO05			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* USB_PWR4 */
	MX6_PAD_GPIO_2__GPIO1_IO02			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* USB_WIFI */
	MX6_PAD_GPIO_7__GPIO1_IO07			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* USB_HUB1_RESET */
	MX6_PAD_EIM_A25__GPIO5_IO02			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* USB_HUB2_RESET */
	MX6_PAD_EIM_CS1__GPIO2_IO24			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* The OC indication from port 2@USB hub 1 */
	MX6_PAD_EIM_D23__GPIO3_IO23			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* The OC indication from port 1@USB hub 1 */
	MX6_PAD_EIM_D24__GPIO3_IO24			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* The OC indication from port 1@USB hub 2 */
	MX6_PAD_EIM_D25__GPIO3_IO25			| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* The OC indication from port 2@USB hub 2 */
	MX6_PAD_EIM_OE__GPIO2_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* The OC indication from USB WIFI */
	MX6_PAD_KEY_COL0__GPIO4_IO06		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* CPLD_RSESERVED1 */
	MX6_PAD_ENET_RXD0__GPIO1_IO27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* CPLD_RSESERVED2 */
	MX6_PAD_ENET_TXD0__GPIO1_IO30		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* VGA_HOTPLUG */
	MX6_PAD_KEY_ROW0__GPIO4_IO07		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* HDMI_INTERRUPT */
	MX6_PAD_KEY_COL1__GPIO4_IO08		| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* HEADPHONE DETECT */
	MX6_PAD_KEY_ROW1__GPIO4_IO09		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* MICROPHONE DETECT */
	MX6_PAD_GPIO_9__GPIO1_IO09			| MUX_PAD_CTRL(NO_PAD_CTRL),

	/* non-gpio mux */
	/* I2C3 */
	MX6_PAD_GPIO_3__I2C3_SCL			| MUX_PAD_CTRL(I2C_PAD_CTRL),
	MX6_PAD_GPIO_6__I2C3_SDA			| MUX_PAD_CTRL(I2C_PAD_CTRL),
};

static void setup_iomux_gpio(void)
{
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));

	gpio_direction_input(PROJECT_DET0);
	gpio_direction_input(PROJECT_DET1);
	gpio_direction_input(PROJECT_DET2);
	gpio_direction_input(PCB_DET0);
	gpio_direction_input(PCB_DET1);
	gpio_direction_input(OC_USB_HUB2_PORT1);
	gpio_direction_input(OC_USB_HUB1_PORT1);
	gpio_direction_input(OC_USB_HUB1_PORT2);
	gpio_direction_input(OC_USB_HUB2_PORT2);
	gpio_direction_input(OC_USB_WIFI);
	gpio_direction_input(EMMC_WRITE_PROTECT);
	gpio_direction_input(HEADPHONE_DET);
	gpio_direction_input(MICROPHONE_DET);
	gpio_direction_input(AUTO_SDN_EVT);
	gpio_direction_input(AUTO_SDN);
}

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
}

iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D4__SD2_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D5__SD2_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D6__SD2_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D7__SD2_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D2__GPIO2_IO02	| MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D0__GPIO2_IO00    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
}

iomux_v3_cfg_t const pcie_pads[] = {
	MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* POWER */
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* RESET */
};

static void setup_pcie(void)
{
	imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));
}

iomux_v3_cfg_t const di0_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		/* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		/* DISP0_VSYNC */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		/*case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;*/
		case 0:
		case 1:
			break;
		case 2:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	setup_pcie();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	setup_iomux_gpio();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

/*#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif*/
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	/* make cpld check t66 still alive */
	t66_cpld_alive_response();

	/* setup t66 power led */
	t66_power_led(1);

	return 0;
}

/*#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(4, 9)) : -1;
}
#endif*/

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

static int project = 0;
static int pcb = 0;
int checkboard(void)
{
	puts("Board: MX6-SabreSD ");

	project = t66_get_project_version();
	switch(project) {
	case 0:
		printf(" t66\n");
		break;
	case 1:
		printf(" t67\n");
		break;
	case 2:
		printf(" A66\n");
		break;
	default:
		printf("unknown");
	}

	return 0;
}

#ifdef CONFIG_MISC_INIT_R
void t66_set_board_version(void)
{
	switch (pcb) {
		case 0:
			setenv("board", "EVT");
			break;
		case 1:
			setenv("board", "DVT");
			break;
		case 2:
		case 3:
			setenv("board", "PVT");
			break;
		default:
			setenv("board", "UND");
			printf("Undetermined board version\n");
	}

	switch (project) {
		case 0:
			setenv("system-product-name", "t66");
			/* we don't have derived product like t66c, just set t66 */
			setenv("product-name", "t66");
			break;
		case 1:
			setenv("system-product-name", "t67");
			setenv("product-name", "t67");
			break;
		case 2:
			setenv("system-product-name", "A66");
			setenv("product-name", "A66");
		default:
			printf("Undetermined project name\n");
	}
	saveenv();
}

static void t66_set_15_sec_circuit(void)
{
	t66_toggle_last_state(LAST_STATE);
	mdelay(500);
}

static int t66_check_first_power_on(void)
{
	int ret = 0;
	/* t66 */
	if (project == 0) {
		if (pcb == 0)
			ret = t66_gpio_read_value(3, 31);
		else if (pcb >= 1)
			ret = t66_gpio_read_value(4, 15);
	}
	/* t67 */
	else if (project == 1)
	{
		ret = t66_gpio_read_value(4, 15);
	}
	/* A66 */
	else if (project == 2)
	{
		ret = t66_gpio_read_value(4, 15);
	}

	return ret;
}

static void t66_check_last_state(void)
{
	char * flag = getenv ("ac-flag");
	char * type = getenv ("ac-type");
	char * init = getenv ("initialized");
#define IS_POWER_RESUME_SET_LAST_STATE	(strncmp(type, "laststate", 9) == 0)
#define IS_POWER_RESUME_SET_POWER_OFF	(strncmp(type, "poweroff", 8) == 0)
#define IS_T66_SHUTDOWN_NORMAL			(strncmp(flag, "0", 1) == 0)
#define IS_T66_FIRST_POWER_ON			(t66_check_first_power_on() == 1)

	/* don't shutdown & open OTG if we use MFGtool to install an empty board */
	if (strncmp(init, "true", 4) != 0 ) {
		return;
	}

	/* start communicate with CPLD */
	t66_set_softoff_pin(GPIO_LOW);

	/* if t66 first poweron */
	if (IS_T66_FIRST_POWER_ON) {
		t66_enable_usb_hub(0);
		mdelay(2000);
		if (IS_T66_SHUTDOWN_NORMAL || IS_POWER_RESUME_SET_POWER_OFF)
			t66_power_off();
		if ((! IS_T66_SHUTDOWN_NORMAL) && IS_POWER_RESUME_SET_LAST_STATE)
			t66_set_15_sec_circuit();
	}

	/* end communicate with CPLD */
	t66_set_softoff_pin(GPIO_HIGH);
}

/* touch reset pin of devices according to EE's request */
void t66_reset_device(void)
{
	/* vga */

	/* lan */
	//t66_reset_pcie();

	/* usb */

	/* audio */
	t66_reset_audio();
	/* emmc */
	t66_reset_emmc();
}

int misc_init_r(void)
{
	pcb = t66_get_pcb_version();
	char * disp_detect = getenv ("disp_detect");
	char * model = getenv ("system-product-name");
	u32 disp_type = 0;

	//t66_check_last_state();
	t66_enable_usb_hub(1);
	t66_reset_device();

	if  ( strncmp(model, "und", 3) == 0 )
		t66_set_board_version();

	if ( strncmp(disp_detect, "on", 2) == 0 ) {
		/* disp_type: 1 ==> DVI, 0 ==> VGA */
		disp_type = t66_gpio_read_value(3, 21);
		if (disp_type == 1)
			setenv("disp0_src", "hdmi");
		else
			setenv("disp0_src", "vga");
	}

	return 0;
}
#endif /* CONFIG_MISC_INIT_R */

int board_late_init(void)
{
	t66_check_last_state();
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}
