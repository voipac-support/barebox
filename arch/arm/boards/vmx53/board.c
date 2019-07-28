/*
 * Copyright (C) 2012 Sascha Hauer, Pengutronix
 * (c) 2014 Voipac <support@voipac.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <common.h>
#include <bootsource.h>
#include <environment.h>
#include <fcntl.h>
#include <fec.h>
#include <fs.h>
#include <init.h>
#include <nand.h>
#include <net.h>
#include <partition.h>
#include <sizes.h>
#include <gpio.h>

#include <generated/mach-types.h>

#include <mach/imx53-regs.h>
#include <mach/iomux-mx53.h>
#include <mach/devices-imx53.h>
#include <mach/generic.h>
#include <mach/imx-nand.h>
#include <mach/iim.h>
#include <mach/imx5.h>
#include <mach/imx-flash-header.h>
#include <mach/bbu.h>
#include <mach/revision.h>

#include <asm/armlinux.h>
#include <io.h>
#include <asm/mmu.h>

#include "mach/imx-gpio.h"


#define VMX53_RESET_OUT_GPIO	IMX_GPIO_NR(7, 12)
#define VMX53_SD1_SEL_GPIO      IMX_GPIO_NR(4,  0)

static iomux_v3_cfg_t vmx53_sys_pads[] = {
	MX53_PAD_GPIO_17__GPIO7_12,	// RESET_OUT
	MX53_PAD_GPIO_10__GPIO4_0,      // SD1_SEL
};

#define IMX_ESD_BASE    	0x63fd9000

static int vmx53_mem_init(void)
{
	uchar   rows, cols, dsiz, banks;
	uint32_t size, esdctl, esdmsc;

	esdctl = readl(IMX_ESD_BASE);
	esdmsc = readl(IMX_ESD_BASE+0x18);

	rows = ((esdctl >> 24) & 7);
	cols = (((esdctl >> 20)+1) & 3);
	dsiz = ((esdctl >> 16) & 1);     // 0=16bit,  1=32bit
	banks = (~(esdmsc >> 5) & 1);    // 0=4banks, 1=8banks
	size = (esdctl)? 1 << (rows + cols + banks + dsiz + 22) : 0;

	if(size)
	{
		if( esdctl & 0x80000000) {
			arm_add_mem_device("ram0", MX53_CSD0_BASE_ADDR, size);

			printk("SDRAM at 0x%08x: %dMB (DDR3 2x%dMb, %d ROWs, %d COLs, %d BANKs)\n",
			 MX53_CSD0_BASE_ADDR, size >> 20, size >> 18,  rows + 11, cols + 8, 4 << banks);
		}
		if( esdctl & 0x40000000) {
			arm_add_mem_device("ram1", MX53_CSD1_BASE_ADDR, size);

			printk("SDRAM at 0x%08x: %dMB (DDR3 2x%dMb, %d ROWs, %d COLs, %d BANKs)\n",
			 MX53_CSD1_BASE_ADDR, size >> 20, size >> 18,  rows + 11, cols + 8, 4 << banks);
		}
	}

	return 0;
}
mem_initcall(vmx53_mem_init);

#ifdef CONFIG_DRIVER_NET_FEC_IMX
#define VMX53_FEC_CLK_GPIO		IMX_GPIO_NR(4, 1)
#define VMX53_FEC_RST_GPIO		IMX_GPIO_NR(4, 2)

static struct vmx53_fec_gpio_setup {
        iomux_v3_cfg_t pad;
        unsigned gpio:9,
                dir:1,
                level:1;
} vmx53_fec_gpios[] = {
        { MX53_PAD_GPIO_12__GPIO4_2,      VMX53_FEC_RST_GPIO, 1, 0, }, /* PHY reset */
        { MX53_PAD_GPIO_11__GPIO4_1,      VMX53_FEC_CLK_GPIO, 1, 1, }, /* PHY CLK enable */
        { MX53_PAD_FEC_REF_CLK__GPIO1_23, IMX_GPIO_NR(1, 23), 0, 0, }, /* ENET_CLK */
        { MX53_PAD_FEC_MDC__GPIO1_31,     IMX_GPIO_NR(1, 31), 1, 0, }, /* MDC */
        { MX53_PAD_FEC_MDIO__GPIO1_22,    IMX_GPIO_NR(1, 22), 1, 0, }, /* MDIO */
        { MX53_PAD_FEC_RXD0__GPIO1_27,    IMX_GPIO_NR(1, 27), 1, 1, }, /* Mode0/RXD0 */
        { MX53_PAD_FEC_RXD1__GPIO1_26,    IMX_GPIO_NR(1, 26), 1, 1, }, /* Mode1/RXD1 */
        { MX53_PAD_FEC_RX_ER__GPIO1_24,   IMX_GPIO_NR(1, 24), 0, 0, }, /* RX_ER */
        { MX53_PAD_FEC_TX_EN__GPIO1_28,   IMX_GPIO_NR(1, 28), 1, 0, }, /* TX_EN */
        { MX53_PAD_FEC_TXD0__GPIO1_30,    IMX_GPIO_NR(1, 30), 1, 0, }, /* TXD0 */
        { MX53_PAD_FEC_TXD1__GPIO1_29,    IMX_GPIO_NR(1, 29), 1, 0, }, /* TXD1 */
        { MX53_PAD_FEC_CRS_DV__GPIO1_25,  IMX_GPIO_NR(1, 25), 1, 1, }, /* Mode2/CRS_DV */
};

static iomux_v3_cfg_t vmx53_fec_pads[] = {
        MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
        MX53_PAD_FEC_MDC__FEC_MDC,
        MX53_PAD_FEC_MDIO__FEC_MDIO,
        MX53_PAD_FEC_RXD0__FEC_RDATA_0,
        MX53_PAD_FEC_RXD1__FEC_RDATA_1,
        MX53_PAD_FEC_RX_ER__FEC_RX_ER,
        MX53_PAD_FEC_TX_EN__FEC_TX_EN,
        MX53_PAD_FEC_TXD0__FEC_TDATA_0,
        MX53_PAD_FEC_TXD1__FEC_TDATA_1,
        MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
};

static inline void vmx53_fec_activate(void)
{
	int i;

	/* Configure LAN8700 pads as GPIO and set up
	 * necessary strap options for PHY
	 */
	for (i = 0; i < ARRAY_SIZE(vmx53_fec_gpios); i++) {
		struct vmx53_fec_gpio_setup *gs = &vmx53_fec_gpios[i];

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
	gpio_set_value(VMX53_FEC_CLK_GPIO, 1);

	/*
	 * Wait some time to let the phy clock stabilise
	 */
	mdelay(1);

	/*
	 * Deassert reset, phy latches the rest of bootstrap pins
	 */
	gpio_set_value(VMX53_FEC_RST_GPIO, 1);

	/* LAN7800 has an internal Power On Reset (POR) signal (OR'ed with
	 * the external RESET signal) which is deactivated 21ms after
	 * power on and latches the strap options.
	 * Delay for 22ms to ensure, that the internal POR is inactive
	 * before reconfiguring the strap pins.
	 */
	mdelay(22);

	/*
	 * The phy is ready, now configure imx53 pads for fec operation
	 */
	mxc_iomux_v3_setup_multiple_pads(vmx53_fec_pads,
			ARRAY_SIZE(vmx53_fec_pads));
}
#else
#define vmx53_fec_activate()	((void)0)
#endif

static inline u64 imx53_uid(void)
{
        int ret;
        u64 uid;

        ret = imx_iim_read(0, 7, &uid, sizeof(uid));
        if (ret != sizeof(uid)) {
		uid = 0;
	}

        return uid;
}

static int vmx53_module_init(void)
{
	if (!of_machine_is_compatible("voipac,imx53-vmx53")) {
		return 0;
	}

	barebox_set_hostname("vmx53");
	armlinux_set_architecture(MACH_TYPE_VMX53);
	armlinux_set_serial(imx53_uid() & 0x7fffffffffffffffull);
	armlinux_set_revision(0x53000 | imx_silicon_revision());

	return 0;
}
fs_initcall(vmx53_module_init);

static int vmx53_console_init(void)
{
	if (!of_machine_is_compatible("voipac,imx53-vmx53")) {
		return 0;
	}

	mxc_iomux_v3_setup_multiple_pads(vmx53_sys_pads, 
		ARRAY_SIZE(vmx53_sys_pads));

	/* assert RESET_OUT_B */
	gpio_direction_output(VMX53_RESET_OUT_GPIO, 0);

	mdelay(10);

	imx53_init_lowlevel(800);
//	imx53_init_lowlevel(1200);

	vmx53_fec_activate();

	/* SDHC 0 = module, 1 = baseboard */
	gpio_direction_output(VMX53_SD1_SEL_GPIO, 1);

	/* deassert RESET_OUT_B */
	gpio_set_value(VMX53_RESET_OUT_GPIO, 1);

	return 0;
}
console_initcall(vmx53_console_init);
