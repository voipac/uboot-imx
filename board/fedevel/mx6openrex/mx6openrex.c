/*
 * Copyright (C) 2015-2017 Voipac.
 *
 * Author: support <support@voipac.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <micrel.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <usb.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_SRE_FAST    | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP  |                     \
                        PAD_CTL_SPEED_HIGH  | PAD_CTL_DSE_80ohm | \
                        PAD_CTL_SRE_FAST    | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_HYS)

#define SPI_PAD_CTRL   (PAD_CTL_HYS         |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL   (PAD_CTL_PUS_100K_UP |                     \
                        PAD_CTL_SPEED_MED   | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_HYS         | PAD_CTL_ODE       | \
                        PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE         | PAD_CTL_PUE       | \
                         PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW | \
                         PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | \
                         PAD_CTL_HYS)

#define RST_PAD_CTRL   (PAD_CTL_ODE         | PAD_CTL_DSE_40ohm | \
                        PAD_CTL_SRE_FAST)

int dram_init(void)
{
        gd->ram_size = PHYS_SDRAM_SIZE;

        return 0;
}

iomux_v3_cfg_t const system_pads[] = {
        MX6_PAD_SD4_CMD__GPIO7_IO09     | MUX_PAD_CTRL(NO_PAD_CTRL), /* Cpu GPIO 0 */
        MX6_PAD_NANDF_RB0__GPIO6_IO10   | MUX_PAD_CTRL(NO_PAD_CTRL), /* Cpu GPIO 1 */
        MX6_PAD_EIM_D22__GPIO3_IO22     | MUX_PAD_CTRL(NO_PAD_CTRL), /* Cpu GPIO 2 */
        MX6_PAD_DI0_PIN4__GPIO4_IO20    | MUX_PAD_CTRL(NO_PAD_CTRL), /* Board variant 0 */
        MX6_PAD_DISP0_DAT7__GPIO4_IO28  | MUX_PAD_CTRL(NO_PAD_CTRL), /* Board variant 1 */
        MX6_PAD_DISP0_DAT14__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), /* Board variant 2 */
        MX6_PAD_DISP0_DAT22__GPIO5_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL), /* Board ID0 */
        MX6_PAD_DISP0_DAT23__GPIO5_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL), /* Board ID1 */
        MX6_PAD_ENET_RX_ER__GPIO1_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL), /* Board ID2 */
        MX6_PAD_SD1_DAT0__GPIO1_IO16    | MUX_PAD_CTRL(RST_PAD_CTRL), /* MCU_RSTINn Open Drain Output */
        MX6_PAD_SD1_CMD__GPIO1_IO18     | MUX_PAD_CTRL(RST_PAD_CTRL), /* MCU_ISPn Open Drain Output */
        MX6_PAD_EIM_WAIT__GPIO5_IO00    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CSPI3_MCU_EN - Unused */
        MX6_PAD_DISP0_DAT12__GPIO5_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), /* Pmic int */
        MX6_PAD_EIM_A25__GPIO5_IO02     | MUX_PAD_CTRL(NO_PAD_CTRL), /* Rstout */
};

#define GPIO_SYSTEM_RSTOUT    IMX_GPIO_NR(5, 2)
#define GPIO_SYSTEM_MCU_RSTIN IMX_GPIO_NR(1, 16)
#define GPIO_SYSTEM_MCU_ISP   IMX_GPIO_NR(1, 18)

static void setup_iomux_system(void)
{
        imx_iomux_v3_setup_multiple_pads(system_pads, ARRAY_SIZE(system_pads));

        gpio_direction_output(GPIO_SYSTEM_RSTOUT, 0);
        gpio_direction_output(GPIO_SYSTEM_MCU_RSTIN, 1);
        gpio_direction_output(GPIO_SYSTEM_MCU_ISP, 1);
}

static void set_system_rstout(int state)
{
        gpio_set_value(GPIO_SYSTEM_RSTOUT, state);
}

iomux_v3_cfg_t const uart1_pads[] = {
        MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
        imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

iomux_v3_cfg_t const enet_pads1[] = {
        MX6_PAD_ENET_MDIO__ENET_MDIO       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_ENET_MDC__ENET_MDC         | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TXC__RGMII_TXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD0__RGMII_TD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD1__RGMII_TD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD2__RGMII_TD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TD3__RGMII_TD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RXC__GPIO6_IO30      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* PHYAD2  = 0 */
        MX6_PAD_RGMII_RD0__GPIO6_IO25      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE0  = 1 */
        MX6_PAD_RGMII_RD1__GPIO6_IO27      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE1  = 1 */
        MX6_PAD_RGMII_RD2__GPIO6_IO28      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE2  = 1 */
        MX6_PAD_RGMII_RD3__GPIO6_IO29      | MUX_PAD_CTRL(ENET_PAD_CTRL), /* MODE3  = 1 */
        MX6_PAD_RGMII_RX_CTL__GPIO6_IO24   | MUX_PAD_CTRL(ENET_PAD_CTRL), /* CLK125_EN = 1 */
        /* KSZ9021 PHY Int */
        MX6_PAD_ENET_TX_EN__GPIO1_IO28     | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_ENET_RXD1__GPIO1_IO26      | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* KSZ9021 PHY Reset */
        MX6_PAD_ENET_CRS_DV__GPIO1_IO25    | MUX_PAD_CTRL(RST_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads2[] = {
        MX6_PAD_RGMII_RXC__RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD0__RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD1__RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD2__RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD3__RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

#define GPIO_ENET_INT_1         IMX_GPIO_NR(1, 28)
#define GPIO_ENET_INT_2         IMX_GPIO_NR(1, 26)
#define GPIO_ENET_RESET         IMX_GPIO_NR(1, 25)
#define GPIO_ENET_CFG_PHYAD2    IMX_GPIO_NR(6, 30)
#define GPIO_ENET_CFG_MODE0     IMX_GPIO_NR(6, 25)
#define GPIO_ENET_CFG_MODE1     IMX_GPIO_NR(6, 27)
#define GPIO_ENET_CFG_MODE2     IMX_GPIO_NR(6, 28)
#define GPIO_ENET_CFG_MODE3     IMX_GPIO_NR(6, 29)
#define GPIO_ENET_CFG_CLK125_EN IMX_GPIO_NR(6, 24)

static void setup_iomux_enet(void)
{
        imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));

        /* KSZ9021 PHY Cfg */
        gpio_direction_output(GPIO_ENET_CFG_PHYAD2,    0);
        gpio_direction_output(GPIO_ENET_CFG_MODE0,     1);
        gpio_direction_output(GPIO_ENET_CFG_MODE1,     1);
        gpio_direction_output(GPIO_ENET_CFG_MODE2,     1);
        gpio_direction_output(GPIO_ENET_CFG_MODE3,     1);
        gpio_direction_output(GPIO_ENET_CFG_CLK125_EN, 1);

        /* KSZ9021 PHY Int */
        gpio_direction_input(GPIO_ENET_INT_1);
        gpio_direction_input(GPIO_ENET_INT_2);

        /* KSZ9021 PHY Reset */
        gpio_direction_output(GPIO_ENET_RESET, 0);
        udelay(10000);
        gpio_set_value(GPIO_ENET_RESET, 1);
        udelay(100);

        imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

iomux_v3_cfg_t const usdhc2_pads[] = {
        MX6_PAD_SD2_CLK__SD2_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD2_CMD__SD2_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_GPIO_4__GPIO1_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
        MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WP */
};

iomux_v3_cfg_t const ecspi1_pads[] = {
        MX6_PAD_EIM_D16__ECSPI1_SCLK    | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_D17__ECSPI1_MISO    | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_D18__ECSPI1_MOSI    | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_EB2__GPIO2_IO30     | MUX_PAD_CTRL(NO_PAD_CTRL), /* CS0 */
        MX6_PAD_DISP0_DAT15__GPIO5_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CS1 */
};

#define GPIO_ECSPI1_CS0     IMX_GPIO_NR(2, 30)
#define GPIO_ECSPI1_CS1     IMX_GPIO_NR(5, 9)

iomux_v3_cfg_t const ecspi2_pads[] = {
        MX6_PAD_DISP0_DAT19__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT17__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT16__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT18__GPIO5_IO12  | MUX_PAD_CTRL(NO_PAD_CTRL), /* CS0 */
};

#define GPIO_ECSPI2_CS0     IMX_GPIO_NR(5, 12)

iomux_v3_cfg_t const ecspi3_pads[] = {
        MX6_PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_DISP0_DAT3__GPIO4_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL), /* CS0 */
        MX6_PAD_DISP0_DAT5__GPIO4_IO26  | MUX_PAD_CTRL(NO_PAD_CTRL), /* CS2 */
};

#define GPIO_ECSPI3_CS0     IMX_GPIO_NR(4, 24)
#define GPIO_ECSPI3_CS2     IMX_GPIO_NR(4, 26)

static void setup_spi(void)
{
        imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
        gpio_direction_output(GPIO_ECSPI1_CS0, 0);
        gpio_direction_output(GPIO_ECSPI1_CS1, 0);

        imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
        gpio_direction_output(GPIO_ECSPI2_CS0, 0);

        imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
        gpio_direction_output(GPIO_ECSPI3_CS0, 0);
        gpio_direction_output(GPIO_ECSPI3_CS2, 0);
}

static struct i2c_pads_info i2c1_pad_info = {
        .scl = {
                .i2c_mode  = MX6_PAD_EIM_D21__I2C1_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(3, 21)
        },
        .sda = {
                .i2c_mode  = MX6_PAD_EIM_D28__I2C1_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(3, 28)
        }
};

static struct i2c_pads_info i2c2_pad_info = {
        .scl = {
                .i2c_mode  = MX6_PAD_KEY_COL3__I2C2_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(4, 12)
        },
        .sda = {
                .i2c_mode  = MX6_PAD_KEY_ROW3__I2C2_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(4, 13)
        }
};

static struct i2c_pads_info i2c3_pad_info = {
        .scl = {
                .i2c_mode  = MX6_PAD_GPIO_5__I2C3_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(1, 5)
        },
        .sda = {
                .i2c_mode  = MX6_PAD_GPIO_16__I2C3_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp        = IMX_GPIO_NR(7, 11)
        }
};

iomux_v3_cfg_t const pcie_pads[] = {
        MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* WAKE */
        MX6_PAD_GPIO_17__GPIO7_IO12      | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* WDIS */
        MX6_PAD_KEY_ROW4__GPIO4_IO15     | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* RESET */
};

static void setup_pcie(void)
{
        imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM];

#define GPIO_USDHC2_CD     IMX_GPIO_NR(1, 4)
#define GPIO_USDHC2_WP     IMX_GPIO_NR(1, 2)

int board_mmc_getcd(struct mmc *mmc)
{
        struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
        int ret = 0;

        switch (cfg->esdhc_base) {
        case USDHC2_BASE_ADDR:
                ret = !gpio_get_value(GPIO_USDHC2_CD);
                break;
        }

        return ret;
}

int board_mmc_init(bd_t *bis)
{
        int ret = 0;
        int i;

        /*
         * According to the board_mmc_init() the following map is done:
         * (U-boot device node)    (Physical Port)
         * mmc0                    SD2 Primary
         */
        for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
                switch (i) {
                case 0:
                        imx_iomux_v3_setup_multiple_pads(
                                usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
                        gpio_direction_input(GPIO_USDHC2_CD);
                        gpio_direction_input(GPIO_USDHC2_WP);
                        usdhc_cfg[0].esdhc_base    = USDHC2_BASE_ADDR;
                        usdhc_cfg[0].sdhc_clk      = mxc_get_clock(MXC_ESDHC2_CLK);
                        usdhc_cfg[0].max_bus_width = 4;
                        break;
                default:
                        printf("Warning: you configured more USDHC controllers"
                               "(%d) then supported by the board (%d)\n",
                               i + 1, CONFIG_SYS_FSL_USDHC_NUM);
                        return -EINVAL;
                }

                ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
        }

        return ret;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
        /* min rx data delay */
        ksz9021_phy_extended_write(phydev,
                        MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
        /* min tx data delay */
        ksz9021_phy_extended_write(phydev,
                        MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
        /* max rx/tx clock delay, min rx/tx control */
        ksz9021_phy_extended_write(phydev,
                        MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
        mx6_rgmii_rework(phydev);

        if (phydev->drv->config)
        {
               phydev->drv->config(phydev);
        }

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

static void enable_hdmi(struct display_info_t const *dev)
{
        disable_lvds(dev);
        imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {
        {
                .bus	= -1,
                .addr	= 0,
                .pixfmt	= IPU_PIX_FMT_RGB24,
                .detect	= NULL,
                .enable	= enable_hdmi,
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
                        .sync           = 0,
                        .vmode          = FB_VMODE_NONINTERLACED
                }
        }
};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
        struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
        struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
        int reg;

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
        if (is_mx6dqp()) {
            int ret;

            /* select ENET MAC0 TX clock from PLL */
            imx_iomux_set_gpr_register(5, 9, 1, 1);
            ret = enable_fec_anatop_clock(0, ENET_125MHZ);
            if (ret)
                printf("Error fec anatop clock settings!\n");
        }

        setup_iomux_enet();

        return cpu_eth_init(bis);
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET 0x800
#define UCTRL_PWR_POL        (1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
        MX6_PAD_KEY_COL4__USB_OTG_OC | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_EIM_D31__GPIO3_IO31  | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_GPIO_1__USB_OTG_ID   | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
};

static iomux_v3_cfg_t const usb_hc1_pads[] = {
        MX6_PAD_GPIO_3__USB_H1_OC | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_usb(void)
{
        imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
            ARRAY_SIZE(usb_otg_pads));

        /*
         * set daisy chain for otg_pin_id on 6q.
         * for 6dl, this bit is reserved
         */
        imx_iomux_set_gpr_register(1, 13, 1, 1);

        imx_iomux_v3_setup_multiple_pads(usb_hc1_pads,
            ARRAY_SIZE(usb_hc1_pads));
}

int board_ehci_hcd_init(int port)
{
        u32 *usbnc_usb_ctrl;

        if (port > 1)
            return -EINVAL;

        usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
            port * 4);

        setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

        return 0;
}

int board_ehci_power(int port, int on)
{
        switch (port) {
        case 0:
            break;
        case 1:
            break;
        default:
            printf("MXC USB port %d not yet supported\n", port);
            return -EINVAL;
        }

        return 0;
}
#endif

void clear_pcie_on_wdog_reset(void)
{
        u32 cause;
        struct src *const src_regs = (struct src *)SRC_BASE_ADDR;
        struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

        cause = readl(&src_regs->srsr);
        if (cause == 0x00010)
        {
                clrbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_REF_SSP_EN);
                clrbits_le32(&iomuxc_regs->gpr[12], IOMUXC_GPR12_APPS_LTSSM_ENABLE);
        }
}

int board_early_init_f(void)
{
		setup_iomux_system();

        set_system_rstout(0);

        setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
        setup_display();
#endif

        clear_pcie_on_wdog_reset();

        return 0;
}

int board_init(void)
{
        /* address of boot parameters */
        gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
        setup_spi();
#endif
        setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c1_pad_info);
        setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c2_pad_info);
        setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c3_pad_info);

        setup_pcie();

#ifdef CONFIG_USB_EHCI_MX6
        setup_usb();
#endif

#if defined(CONFIG_CMD_SATA) && (defined(CONFIG_MX6Q) || defined(CONFIG_MX6QP))
        setup_sata();
#endif

        return 0;
}

static struct pmic *pfuze = NULL;


int power_init_board(void)
{
#if defined(CONFIG_POWER_PFUZE100)
        unsigned int value;
        int ret;

        ret = power_pfuze100_init(0);
        if (ret) {
            return -ENODEV;
        }

        pfuze = pmic_get("PFUZE100");
        ret = pmic_probe(pfuze);
        if (ret) {
            pfuze = NULL;
            return 0;
        }

        pmic_reg_read(pfuze, PFUZE100_DEVICEID, &value);
        printf("PMIC:  PFUZE100 ID=0x%02x\n", value);

        /* Set SW1AB stanby volage to 0.975V */
        pmic_reg_read(pfuze, PFUZE100_SW1ABSTBY, &value);
        value &= ~SW1x_STBY_MASK;
        value |= SW1x_0_975V;
        pmic_reg_write(pfuze, PFUZE100_SW1ABSTBY, value);

        /* Set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
        pmic_reg_read(pfuze, PFUZE100_SW1ABCONF, &value);
        value &= ~SW1xCONF_DVSSPEED_MASK;
        value |= SW1xCONF_DVSSPEED_4US;
        pmic_reg_write(pfuze, PFUZE100_SW1ABCONF, value);

        if (is_mx6dqp()) {
                /* set SW1C staby volatage 1.075V*/
                pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &value);
                value &= ~0x3f;
                value |= 0x1f;
                pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, value);

                /* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
                pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &value);
                value &= ~0xc0;
                value |= 0x40;
                pmic_reg_write(pfuze, PFUZE100_SW1CCONF, value);
        } else {
                /* set SW1C staby volatage 0.975V*/
                pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &value);
                value &= ~0x3f;
                value |= 0x1b;
                pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, value);

                /* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
                pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &value);
                value &= ~0xc0;
                value |= 0x40;
                pmic_reg_write(pfuze, PFUZE100_SW1CCONF, value);
        }
#endif
        return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
        struct pmic *p = pfuze;

        if (!p) {
                printf("No PMIC found!\n");
                return;
        }
#if defined(CONFIG_POWER_PFUZE100)
        /* increase VDDARM/VDDSOC to support 1.2G chip */
        if (check_1_2G()) {
                /* ldo_enable on 1.2G chip */
                if (ldo_bypass)
                {
                        printf("1.2G chip, increase VDDARM/VDDSOC and disable ldo_bypass!\n");
                }
                else
                {
//                        unsigned int value;
//                        /* increase VDDARM to 1.425V */
//                        pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
//                        value &= ~0x3f;
//                        value |= 0x2d;
//                        pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
//
//                        /* increase VDDSOC to 1.425V */
//                        pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
//                        value &= ~0x3f;
//                        value |= 0x2d;
//                        pmic_reg_write(p, PFUZE100_SW1CVOL, value);
                }
        }
#endif
}
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        int ret = -1;

        if (bus == 0 && cs == 0) ret = GPIO_ECSPI1_CS0;
        if (bus == 0 && cs == 1) ret = GPIO_ECSPI1_CS1;
        if (bus == 1 && cs == 0) ret = GPIO_ECSPI2_CS0;
        if (bus == 2 && cs == 0) ret = GPIO_ECSPI3_CS0;
        if (bus == 2 && cs == 2) ret = GPIO_ECSPI3_CS2;

        return ret;
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
        /* 4 bit bus width */
        {"mmc0", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)}, // SD2
        {NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
        add_board_boot_modes(board_boot_modes);
#endif

        set_system_rstout(1);

        return 0;
}

int checkboard(void)
{
        puts("Board: MX6 OpenRex - " CONFIG_MODULE_TYPE_POSTFIX "\n");
        return 0;
}
