/*
 * (C) 2011 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 * (C) 2014-2017 Voipac <support@voipac.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 */

#define pr_fmt(fmt) "vmx25: " fmt

#include <common.h>
#include <init.h>
#include <driver.h>
#include <linux/sizes.h>
#include <envfs.h>
#include <gpio.h>
#include <environment.h>
#include <mach/imx25-regs.h>
#include <mach/esdctl.h>
#include <asm/armlinux.h>
#include <asm/sections.h>
#include <asm/barebox-arm.h>
#include <io.h>
#include <partition.h>
#include <generated/mach-types.h>
#include <mach/imx-nand.h>
#include <mach/iomux-mx25.h>
#include <mach/generic.h>
#include <mach/iim.h>
#include <linux/err.h>
#include <mach/devices-imx25.h>
#include <mach/bbu.h>
#include <asm/mmu.h>

#define VMX25_RESET_OUT_GPIO	IMX_GPIO_NR(3, 18)
#define VMX25_SD1_SEL_GPIO		IMX_GPIO_NR(4, 10)

static iomux_v3_cfg_t vmx25_sys_pads[] = {
	MX25_PAD_VSTBY_ACK__GPIO_3_18,	// RESET_OUT
	MX25_PAD_D10__GPIO_4_10,		// SD1_SEL
};

static int vmx25_mem_init(void)
{
	uint32_t rows, cols, size;
	uint32_t esdctl;
	uint32_t esdctl0reg = MX25_ESDCTL_BASE_ADDR + IMX_ESDCTL0;
	uint32_t esdctl1reg = MX25_ESDCTL_BASE_ADDR + IMX_ESDCTL1;

	esdctl = readl(esdctl0reg);
	rows = (esdctl & ESDCTL0_ROW_MASK) >> 24;
	cols = (esdctl & ESDCTL0_COL_MASK) >> 20;
	size = (esdctl) ? 1 << (rows + cols + 22) : 0;

	printk("SDRAM at 0x%08x: %dMB (%dMb, %d ROWs, %d COLs)\n",
		 MX25_CSD0_BASE_ADDR, size >> 20, size >> 17, rows + 11, cols + 8);

	esdctl = readl(esdctl1reg);
	rows = (esdctl & ESDCTL0_ROW_MASK) >> 24;
	cols = (esdctl & ESDCTL0_COL_MASK) >> 20;
	size = (esdctl) ? 1 << (rows + cols + 22) : 0;

	printk("SDRAM at 0x%08x: %dMB (%dMb, %d ROWs, %d COLs)\n",
		 MX25_CSD1_BASE_ADDR, size >> 20, size >> 17, rows + 11, cols + 8);

	return 0;
}
mem_initcall(vmx25_mem_init);

#ifdef CONFIG_DRIVER_NET_FEC_IMX
#define VMX25_FEC_CLK_GPIO		IMX_GPIO_NR(4, 9)
#define VMX25_FEC_RST_GPIO		IMX_GPIO_NR(4, 7)

static struct vmx25_fec_gpio_setup {
	iomux_v3_cfg_t pad;
	unsigned gpio:9,
	dir:1,
	level:1;
} vmx25_fec_gpios[] = {
	{ MX25_PAD_D13__GPIO_4_7,         VMX25_FEC_RST_GPIO, 1, 0, }, /* PHY reset */
	{ MX25_PAD_D11__GPIO_4_9,         VMX25_FEC_CLK_GPIO, 1, 1, }, /* PHY CLK enable */
	{ MX25_PAD_FEC_TX_CLK__GPIO_3_13, IMX_GPIO_NR(3, 13), 0, 0, }, /* ENET_CLK */
	{ MX25_PAD_FEC_MDC__GPIO_3_5,     IMX_GPIO_NR(3,  5), 1, 0, }, /* MDC */
	{ MX25_PAD_FEC_MDIO__GPIO_3_6,    IMX_GPIO_NR(3,  6), 1, 0, }, /* MDIO */
	{ MX25_PAD_FEC_RDATA0__GPIO_3_10, IMX_GPIO_NR(3, 10), 1, 1, }, /* Mode0/RXD0 */
	{ MX25_PAD_FEC_RDATA1__GPIO_3_11, IMX_GPIO_NR(3, 11), 1, 1, }, /* Mode1/RXD1 */
	{ MX25_PAD_D12__GPIO_4_8,         IMX_GPIO_NR(4,  8), 0, 0, }, /* RX_ER */
	{ MX25_PAD_FEC_TX_EN__GPIO_3_9,   IMX_GPIO_NR(3,  9), 1, 0, }, /* TX_EN */
	{ MX25_PAD_FEC_TDATA0__GPIO_3_7,  IMX_GPIO_NR(3,  7), 1, 0, }, /* TXD0 */
	{ MX25_PAD_FEC_TDATA1__GPIO_3_8,  IMX_GPIO_NR(3,  8), 1, 0, }, /* TXD1 */
	{ MX25_PAD_FEC_RX_DV__GPIO_3_12,  IMX_GPIO_NR(3, 12), 1, 1, }, /* Mode2/CRS_DV */
};

static iomux_v3_cfg_t vmx25_fec_pads[] = {
	MX25_PAD_FEC_TX_CLK__FEC_TX_CLK,
	MX25_PAD_FEC_MDC__FEC_MDC,
	MX25_PAD_FEC_MDIO__FEC_MDIO,
	MX25_PAD_FEC_RDATA0__FEC_RDATA0,
	MX25_PAD_FEC_RDATA1__FEC_RDATA1,
	MX25_PAD_FEC_TX_EN__FEC_TX_EN,
	MX25_PAD_FEC_TDATA0__FEC_TDATA0,
	MX25_PAD_FEC_TDATA1__FEC_TDATA1,
	MX25_PAD_FEC_RX_DV__FEC_RX_DV,
};

static void noinline vmx25_fec_activate(void)
{
	int i;

	/* Configure LAN8700 pads as GPIO and set up
	 * necessary strap options for PHY
	 */
	for (i = 0; i < ARRAY_SIZE(vmx25_fec_gpios); i++) {
		struct vmx25_fec_gpio_setup *gs = &vmx25_fec_gpios[i];

		if (gs->dir) {
			gpio_direction_output(gs->gpio, gs->level);
		} else {
			gpio_direction_input(gs->gpio);
		}

		mxc_iomux_v3_setup_pad(gs->pad);
	}

	/*
	 *Turn on phy clk, leave in reset state
	 */
	gpio_set_value(VMX25_FEC_CLK_GPIO, 1);

	/*
	 * Wait some time to let the phy clock stabilise
	 */
	mdelay(1);

	/*
	 * Deassert reset, phy latches the rest of bootstrap pins
	 */
	gpio_set_value(VMX25_FEC_RST_GPIO, 1);

	/* LAN7800 has an internal Power On Reset (POR) signal (OR'ed with
	 * the external RESET signal) which is deactivated 21ms after
	 * power on and latches the strap options.
	 * Delay for 22ms to ensure, that the internal POR is inactive
	 * before reconfiguring the strap pins.
	 */
	mdelay(22);

	/*
	 * The phy is ready, now configure imx25 pads for fec operation
	 */
	mxc_iomux_v3_setup_multiple_pads(vmx25_fec_pads,
			ARRAY_SIZE(vmx25_fec_pads));

	/* FEC driver picks up the reset gpio later */
	gpio_free(VMX25_FEC_RST_GPIO);
}
#else
#define vmx25_fec_activate()	((void)0)
#endif

#undef IIM_SREV
#define IIM_SREV 0x24

static unsigned int imx25_silicon_revision(void)
{
	void __iomem *iim_base = IOMEM(MX25_IIM_BASE_ADDR);
	u32 rev = readl(iim_base + IIM_SREV) & 0xff;

	return (unsigned int) rev;
}

static int vmx25_module_init(void)
{
	if (!of_machine_is_compatible("voipac,imx25-vmx25")) {
		return 0;
	}

	barebox_set_hostname("vmx25");
	armlinux_set_architecture(MACH_TYPE_VMX25);
	armlinux_set_serial(imx_uid());
	armlinux_set_revision(0x25000|imx25_silicon_revision());

	if (IS_ENABLED(CONFIG_DEFAULT_ENVIRONMENT_GENERIC_NEW)) {
		defaultenv_append_directory(defaultenv_vmx25);
	}

	return 0;
}
fs_initcall(vmx25_module_init);

static int vmx25_console_init(void)
{
	if (!of_machine_is_compatible("voipac,imx25-vmx25")) {
		return 0;
	}

	mxc_iomux_v3_setup_multiple_pads(vmx25_sys_pads, 
		ARRAY_SIZE(vmx25_sys_pads));

	/* assert RESET_OUT_B */
	gpio_direction_output(VMX25_RESET_OUT_GPIO, 0);

	mdelay(10);

	/* SDHC 0 = baseboard, 1 = module */
	gpio_set_value(VMX25_SD1_SEL_GPIO, 0);

	vmx25_fec_activate();

	imx_bbu_external_nand_register_handler("nand", "/dev/nand0.boot",
			BBU_HANDLER_FLAG_DEFAULT);

	/* deassert RESET_OUT_B */
	gpio_set_value(VMX25_RESET_OUT_GPIO, 1);

	return 0;
}
console_initcall(vmx25_console_init);

#define VMX25_LCD_BCKLIGHT_GPIO	IMX_GPIO_NR(1, 26)
#define VMX25_LCD_POWER_GPIO	IMX_GPIO_NR(4, 3)

static iomux_v3_cfg_t vmx25_lcd_pads[] = {
	/* LCDC */
	MX25_PAD_LD0__LD0,
	MX25_PAD_LD1__LD1,
	MX25_PAD_LD2__LD2,
	MX25_PAD_LD3__LD3,
	MX25_PAD_LD4__LD4,
	MX25_PAD_LD5__LD5,
	MX25_PAD_LD6__LD6,
	MX25_PAD_LD7__LD7,
	MX25_PAD_LD8__LD8,
	MX25_PAD_LD9__LD9,
	MX25_PAD_LD10__LD10,
	MX25_PAD_LD11__LD11,
	MX25_PAD_LD12__LD12,
	MX25_PAD_LD13__LD13,
	MX25_PAD_LD14__LD14,
	MX25_PAD_LD15__LD15,
	MX25_PAD_GPIO_E__LD16,
	MX25_PAD_GPIO_F__LD17,
	MX25_PAD_LSCLK__LSCLK,
	MX25_PAD_OE_ACD__OE_ACD,
	MX25_PAD_VSYNC__VSYNC,
	MX25_PAD_HSYNC__HSYNC,
	// BACKLIGHT CONTROL
	MX25_PAD_PWM__GPIO_1_26,
	// PSAVE
	MX25_PAD_CS1__GPIO_4_3,
};

static struct imx_fb_videomode vmx25_fb_mode = {
	.bpp	= 16,
	.mode = {
		.name         = "VGA",
		.pixclock     = 30076,

		.xres         = 640,
		.yres         = 480,

		.hsync_len    = 64,
		.left_margin  = 96,
		.right_margin = 48,

		.vsync_len    = 2,
		.upper_margin = 33,
		.lower_margin = 10,
	},
	.pcr	= PCR_TFT | PCR_COLOR | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
};

static void vmx25_fb_enable(int enable)
{
	gpio_direction_output(VMX25_LCD_POWER_GPIO, 1);

	if (enable) {
		gpio_direction_output(VMX25_LCD_BCKLIGHT_GPIO, 0);
	} else {
		gpio_direction_output(VMX25_LCD_BCKLIGHT_GPIO, 1);
	}
}

static struct imx_fb_platform_data vmx25_fb_data = {
	.mode		= &vmx25_fb_mode,
	.num_modes	= 1,
	.dmacr		= 0x80040060,
	.enable		= vmx25_fb_enable,
};

static int vmx25_init_fb(void)
{
	if (!IS_ENABLED(CONFIG_DRIVER_VIDEO_IMX))
		return 0;

	if (!of_machine_is_compatible("voipac,imx25-vmx25"))
		return 0;

	mxc_iomux_v3_setup_multiple_pads(vmx25_lcd_pads,
			ARRAY_SIZE(vmx25_lcd_pads));

	vmx25_fb_enable(0);

	imx25_add_fb(&vmx25_fb_data);

	return 0;
}
device_initcall(vmx25_init_fb);
