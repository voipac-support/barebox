/*
 * ocotp.c - i.MX6 ocotp fusebox driver
 *
 * Provide an interface for programming and sensing the information that are
 * stored in on-chip fuse elements. This functionality is part of the IC
 * Identification Module (IIM), which is present on some i.MX CPUs.
 *
 * Copyright (c) 2010 Baruch Siach <baruch@tkos.co.il>,
 * 	Orex Computed Radiography
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <common.h>
#include <driver.h>
#include <malloc.h>
#include <xfuncs.h>
#include <errno.h>
#include <init.h>
#include <net.h>
#include <io.h>
#include <of.h>
#include <clock.h>
#include <regmap.h>
#include <linux/clk.h>
#include <mach/ocotp.h>

/*
 * a single MAC address reference has the form
 * <&phandle regoffset>
 */
#define MAC_ADDRESS_PROPLEN	(2 * sizeof(__be32))

/* OCOTP Registers offsets */
#define OCOTP_CTRL			0x00
#define OCOTP_CTRL_SET			0x04
#define OCOTP_CTRL_CLR			0x08
#define OCOTP_TIMING			0x10
#define OCOTP_DATA			0x20
#define OCOTP_READ_CTRL			0x30
#define OCOTP_READ_FUSE_DATA		0x40

/* OCOTP Registers bits and masks */
#define OCOTP_CTRL_WR_UNLOCK		16
#define OCOTP_CTRL_WR_UNLOCK_KEY	0x3E77
#define OCOTP_CTRL_WR_UNLOCK_MASK	0xFFFF0000
#define OCOTP_CTRL_ADDR			0
#define OCOTP_CTRL_ADDR_MASK		0x0000007F
#define OCOTP_CTRL_BUSY			(1 << 8)
#define OCOTP_CTRL_ERROR		(1 << 9)
#define OCOTP_CTRL_RELOAD_SHADOWS	(1 << 10)

#define OCOTP_TIMING_STROBE_READ	16
#define OCOTP_TIMING_STROBE_READ_MASK	0x003F0000
#define OCOTP_TIMING_RELAX		12
#define OCOTP_TIMING_RELAX_MASK		0x0000F000
#define OCOTP_TIMING_STROBE_PROG	0
#define OCOTP_TIMING_STROBE_PROG_MASK	0x00000FFF

#define OCOTP_READ_CTRL_READ_FUSE	0x00000001

#define BF(value, field)		(((value) << field) & field##_MASK)

/* Other definitions */
#define IMX6_OTP_DATA_ERROR_VAL		0xBADABADA
#define DEF_RELAX			20
#define MAC_OFFSET_0			(0x22 * 4)
#define MAC_OFFSET_1			(0x24 * 4)
#define MAX_MAC_OFFSETS			2
#define MAC_BYTES			8

struct imx_ocotp_data {
	int num_regs;
	u32 (*addr_to_offset)(u32 addr);
	u8  mac_offsets[MAX_MAC_OFFSETS];
	u8  mac_offsets_num;
};

struct ocotp_priv_ethaddr {
	char value[MAC_BYTES];
	struct regmap *map;
	u8 offset;
};

struct ocotp_priv {
	struct regmap *map;
	void __iomem *base;
	struct clk *clk;
	struct device_d dev;
	int permanent_write_enable;
	int sense_enable;
	struct ocotp_priv_ethaddr ethaddr[MAX_MAC_OFFSETS];
	struct regmap_config map_config;
	const struct imx_ocotp_data *data;
	int  mac_offset_idx;
};

static struct ocotp_priv *imx_ocotp;

static int imx6_ocotp_set_timing(struct ocotp_priv *priv)
{
	u32 clk_rate;
	u32 relax, strobe_read, strobe_prog;
	u32 timing;

	clk_rate = clk_get_rate(priv->clk);

	relax = clk_rate / (1000000000 / DEF_RELAX) - 1;
	strobe_prog = clk_rate / (1000000000 / 10000) + 2 * (DEF_RELAX + 1) - 1;
	strobe_read = clk_rate / (1000000000 / 40) + 2 * (DEF_RELAX + 1) - 1;

	timing = BF(relax, OCOTP_TIMING_RELAX);
	timing |= BF(strobe_read, OCOTP_TIMING_STROBE_READ);
	timing |= BF(strobe_prog, OCOTP_TIMING_STROBE_PROG);

	writel(timing, priv->base + OCOTP_TIMING);

	return 0;
}

static int imx6_ocotp_wait_busy(struct ocotp_priv *priv, u32 flags)
{
	uint64_t start = get_time_ns();

	while (readl(priv->base + OCOTP_CTRL) & (OCOTP_CTRL_BUSY | flags))
		if (is_timeout(start, MSECOND))
			return -ETIMEDOUT;

	return 0;
}

static int imx6_ocotp_prepare(struct ocotp_priv *priv)
{
	int ret;

	ret = imx6_ocotp_set_timing(priv);
	if (ret)
		return ret;

	ret = imx6_ocotp_wait_busy(priv, 0);
	if (ret)
		return ret;

	return 0;
}

static int fuse_read_addr(struct ocotp_priv *priv, u32 addr, u32 *pdata)
{
	u32 ctrl_reg;
	int ret;

	writel(OCOTP_CTRL_ERROR, priv->base + OCOTP_CTRL_CLR);

	ctrl_reg = readl(priv->base + OCOTP_CTRL);
	ctrl_reg &= ~OCOTP_CTRL_ADDR_MASK;
	ctrl_reg &= ~OCOTP_CTRL_WR_UNLOCK_MASK;
	ctrl_reg |= BF(addr, OCOTP_CTRL_ADDR);
	writel(ctrl_reg, priv->base + OCOTP_CTRL);

	writel(OCOTP_READ_CTRL_READ_FUSE, priv->base + OCOTP_READ_CTRL);
	ret = imx6_ocotp_wait_busy(priv, 0);
	if (ret)
		return ret;

	if (readl(priv->base + OCOTP_CTRL) & OCOTP_CTRL_ERROR)
		*pdata = 0xbadabada;
	else
		*pdata = readl(priv->base + OCOTP_READ_FUSE_DATA);

	return 0;
}

int imx6_ocotp_read_one_u32(struct ocotp_priv *priv, u32 index, u32 *pdata)
{
	int ret;

	ret = imx6_ocotp_prepare(priv);
	if (ret) {
		dev_err(&priv->dev, "failed to prepare read fuse 0x%08x\n",
				index);
		return ret;
	}

	ret = fuse_read_addr(priv, index, pdata);
	if (ret) {
		dev_err(&priv->dev, "failed to read fuse 0x%08x\n", index);
		return ret;
	}

	return 0;
}

static int imx_ocotp_reg_read(void *ctx, unsigned int reg, unsigned int *val)
{
	struct ocotp_priv *priv = ctx;
	u32 index;
	int ret;

	index = reg >> 2;

	if (priv->sense_enable) {
		ret = imx6_ocotp_read_one_u32(priv, index, val);
		if (ret)
			return ret;
	} else {
		*(u32 *)val = readl(priv->base +
				    priv->data->addr_to_offset(index));
	}

	return 0;
}

static int fuse_blow_addr(struct ocotp_priv *priv, u32 addr, u32 value)
{
	u32 ctrl_reg;
	int ret;

	writel(OCOTP_CTRL_ERROR, priv->base + OCOTP_CTRL_CLR);

	/* Control register */
	ctrl_reg = readl(priv->base + OCOTP_CTRL);
	ctrl_reg &= ~OCOTP_CTRL_ADDR_MASK;
	ctrl_reg |= BF(addr, OCOTP_CTRL_ADDR);
	ctrl_reg |= BF(OCOTP_CTRL_WR_UNLOCK_KEY, OCOTP_CTRL_WR_UNLOCK);
	writel(ctrl_reg, priv->base + OCOTP_CTRL);

	writel(value, priv->base + OCOTP_DATA);
	ret = imx6_ocotp_wait_busy(priv, 0);
	if (ret)
		return ret;

	/* Write postamble */
	udelay(2000);
	return 0;
}

static int imx6_ocotp_reload_shadow(struct ocotp_priv *priv)
{
	dev_info(&priv->dev, "reloading shadow registers...\n");
	writel(OCOTP_CTRL_RELOAD_SHADOWS, priv->base + OCOTP_CTRL_SET);
	udelay(1);

	return imx6_ocotp_wait_busy(priv, OCOTP_CTRL_RELOAD_SHADOWS);
}

int imx6_ocotp_blow_one_u32(struct ocotp_priv *priv, u32 index, u32 data,
			    u32 *pfused_value)
{
	int ret;

	ret = imx6_ocotp_prepare(priv);
	if (ret) {
		dev_err(&priv->dev, "prepare to write failed\n");
		return ret;
	}

	ret = fuse_blow_addr(priv, index, data);
	if (ret) {
		dev_err(&priv->dev, "fuse blow failed\n");
		return ret;
	}

	if (readl(priv->base + OCOTP_CTRL) & OCOTP_CTRL_ERROR) {
		dev_err(&priv->dev, "bad write status\n");
		return -EFAULT;
	}

	ret = imx6_ocotp_read_one_u32(priv, index, pfused_value);

	return ret;
}

static int imx_ocotp_reg_write(void *ctx, unsigned int reg, unsigned int val)
{
	struct ocotp_priv *priv = ctx;
	int index;
	u32 pfuse;
	int ret;

	index = reg >> 2;

	if (priv->permanent_write_enable) {
		ret = imx6_ocotp_blow_one_u32(priv, index, val, &pfuse);
		if (ret < 0)
			return ret;
	} else {
		writel(val, priv->base +
		       priv->data->addr_to_offset(index));
	}

	if (priv->permanent_write_enable)
		imx6_ocotp_reload_shadow(priv);

	return 0;
}

static void imx_ocotp_field_decode(uint32_t field, unsigned *word,
				 unsigned *bit, unsigned *mask)
{
	unsigned width;

	*word = ((field >> OCOTP_WORD_MASK_SHIFT) & ((1 << OCOTP_WORD_MASK_WIDTH) - 1)) * 4;
	*bit = (field >> OCOTP_BIT_MASK_SHIFT) & ((1 << OCOTP_BIT_MASK_WIDTH) - 1);
	width = ((field >> OCOTP_WIDTH_MASK_SHIFT) & ((1 << OCOTP_WIDTH_MASK_WIDTH) - 1)) + 1;
	*mask = (1 << width) - 1;
}

int imx_ocotp_read_field(uint32_t field, unsigned *value)
{
	unsigned word, bit, mask, val;
	int ret;

	imx_ocotp_field_decode(field, &word, &bit, &mask);

	ret = imx_ocotp_reg_read(imx_ocotp, word, &val);
	if (ret)
		return ret;

	val >>= bit;
	val &= mask;

	dev_dbg(&imx_ocotp->dev, "%s: word: 0x%x bit: %d mask: 0x%x val: 0x%x\n",
		__func__, word, bit, mask, val);

	*value = val;

	return 0;
}

int imx_ocotp_write_field(uint32_t field, unsigned value)
{
	unsigned word, bit, mask;
	int ret;

	imx_ocotp_field_decode(field, &word, &bit, &mask);

	value &= mask;
	value <<= bit;

	ret = imx_ocotp_reg_write(imx_ocotp, word, value);
	if (ret)
		return ret;

	dev_dbg(&imx_ocotp->dev, "%s: word: 0x%x bit: %d mask: 0x%x val: 0x%x\n",
		__func__, word, bit, mask, value);

	return 0;
}

int imx_ocotp_permanent_write(int enable)
{
	imx_ocotp->permanent_write_enable = enable;

	return 0;
}

bool imx_ocotp_sense_enable(bool enable)
{
	const bool old_value = imx_ocotp->sense_enable;
	imx_ocotp->sense_enable = enable;
	return old_value;
}

static uint32_t inc_offset(uint32_t offset)
{
	if ((offset & 0x3) == 0x3)
		return offset + 0xd;
	else
		return offset + 1;
}

static void imx_ocotp_init_dt(struct device_d *dev, void __iomem *base)
{
	char mac[6];
	const __be32 *prop;
	struct device_node *node = dev->device_node;
	int len;

	if (!node)
		return;

	prop = of_get_property(node, "barebox,provide-mac-address", &len);
	if (!prop)
		return;

	while (len >= MAC_ADDRESS_PROPLEN) {
		struct device_node *rnode;
		uint32_t phandle, offset;

		phandle = be32_to_cpup(prop++);

		rnode = of_find_node_by_phandle(phandle);
		offset = be32_to_cpup(prop++);

		mac[5] = readb(base + offset);
		offset = inc_offset(offset);
		mac[4] = readb(base + offset);
		offset = inc_offset(offset);
		mac[3] = readb(base + offset);
		offset = inc_offset(offset);
		mac[2] = readb(base + offset);
		offset = inc_offset(offset);
		mac[1] = readb(base + offset);
		offset = inc_offset(offset);
		mac[0] = readb(base + offset);

		of_eth_register_ethaddr(rnode, mac);

		len -= MAC_ADDRESS_PROPLEN;
	}
}

static void memreverse(void *dest, const void *src, size_t n)
{
	char *destp = dest;
	const char *srcp = src + n - 1;

	while(n--)
		*destp++ = *srcp--;
}

static int imx_ocotp_get_mac(struct param_d *param, void *priv)
{
	char buf[MAC_BYTES];
	int ret;
	struct ocotp_priv_ethaddr *ethaddr = priv;

	ret = regmap_bulk_read(ethaddr->map, ethaddr->offset,
			       buf, MAC_BYTES);
	if (ret < 0)
		return ret;

	memreverse(ethaddr->value, buf, 6);
	return 0;
}

static int imx_ocotp_set_mac(struct param_d *param, void *priv)
{
	char buf[MAC_BYTES];
	struct ocotp_priv_ethaddr *ethaddr = priv;

	memreverse(buf, ethaddr->value, 6);

	return regmap_bulk_write(ethaddr->map, ethaddr->offset,
				 buf, MAC_BYTES);
}

static struct regmap_bus imx_ocotp_regmap_bus = {
	.reg_write = imx_ocotp_reg_write,
	.reg_read = imx_ocotp_reg_read,
};

static int imx_ocotp_probe(struct device_d *dev)
{
	struct resource *iores;
	void __iomem *base;
	struct ocotp_priv *priv;
	int ret = 0;
	const struct imx_ocotp_data *data;

	ret = dev_get_drvdata(dev, (const void **)&data);
	if (ret)
		return ret;

	iores = dev_request_mem_resource(dev, 0);
	if (IS_ERR(iores))
		return PTR_ERR(iores);
	base = IOMEM(iores->start);

	imx_ocotp_init_dt(dev, base);

	priv = xzalloc(sizeof(*priv));

	priv->data      = data;
	priv->base	= base;
	priv->clk	= clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	strcpy(priv->dev.name, "ocotp");
	priv->dev.parent = dev;
	register_device(&priv->dev);

	priv->map_config.reg_bits = 32;
	priv->map_config.val_bits = 32;
	priv->map_config.reg_stride = 4;
	priv->map_config.max_register = data->num_regs - 1;

	priv->map = regmap_init(dev, &imx_ocotp_regmap_bus, priv, &priv->map_config);
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	ret = regmap_register_cdev(priv->map, "imx-ocotp");
	if (ret)
		return ret;

	imx_ocotp = priv;

	if (IS_ENABLED(CONFIG_IMX_OCOTP_WRITE)) {
		dev_add_param_bool(&(priv->dev), "permanent_write_enable",
				NULL, NULL, &priv->permanent_write_enable, NULL);
	}

	if (IS_ENABLED(CONFIG_NET)) {
		int i;
		struct ocotp_priv_ethaddr *ethaddr;

		for (i = 0; i < priv->data->mac_offsets_num; i++) {
			ethaddr = &priv->ethaddr[i];
			ethaddr->map = priv->map;
			ethaddr->offset = priv->data->mac_offsets[i];

			dev_add_param_mac(&priv->dev, xasprintf("mac_addr%d", i),
					  imx_ocotp_set_mac, imx_ocotp_get_mac,
					  ethaddr->value, ethaddr);
		}

		/*
		 * Alias to mac_addr0 for backwards compatibility
		 */
		ethaddr = &priv->ethaddr[0];
		dev_add_param_mac(&priv->dev, "mac_addr",
				  imx_ocotp_set_mac, imx_ocotp_get_mac,
				  ethaddr->value, ethaddr);
	}

	dev_add_param_bool(&(priv->dev), "sense_enable", NULL, NULL, &priv->sense_enable, priv);

	return 0;
}

static u32 imx6sl_addr_to_offset(u32 addr)
{
	return 0x400 + addr * 0x10;
}

static u32 imx6q_addr_to_offset(u32 addr)
{
	u32 addendum = 0;

	if (addr > 0x2F) {
		/*
		 * If we are reading past Bank 5, take into account a
		 * 0x100 bytes wide gap between Bank 5 and Bank 6
		 */
		addendum += 0x100;
	}


	return imx6sl_addr_to_offset(addr) + addendum;
}

static u32 vf610_addr_to_offset(u32 addr)
{
	if (addr == 0x04)
		return 0x450;
	else
		return imx6q_addr_to_offset(addr);
}

static struct imx_ocotp_data imx6q_ocotp_data = {
	.num_regs = 512,
	.addr_to_offset = imx6q_addr_to_offset,
	.mac_offsets_num = 1,
	.mac_offsets = { MAC_OFFSET_0 },
};

static struct imx_ocotp_data imx6sl_ocotp_data = {
	.num_regs = 256,
	.addr_to_offset = imx6sl_addr_to_offset,
	.mac_offsets_num = 1,
	.mac_offsets = { MAC_OFFSET_0 },
};

static struct imx_ocotp_data vf610_ocotp_data = {
	.num_regs = 512,
	.addr_to_offset = vf610_addr_to_offset,
	.mac_offsets_num = 2,
	.mac_offsets = { MAC_OFFSET_0, MAC_OFFSET_1 },
};

static __maybe_unused struct of_device_id imx_ocotp_dt_ids[] = {
	{
		.compatible = "fsl,imx6q-ocotp",
		.data = &imx6q_ocotp_data,
	}, {
		.compatible = "fsl,imx6sx-ocotp",
		.data = &imx6q_ocotp_data,
	}, {
		.compatible = "fsl,imx6sl-ocotp",
		.data = &imx6sl_ocotp_data,
	}, {
		.compatible = "fsl,imx6ul-ocotp",
		.data = &imx6q_ocotp_data,
	}, {
		.compatible = "fsl,vf610-ocotp",
		.data = &vf610_ocotp_data,
	}, {
		/* sentinel */
	}
};

static struct driver_d imx_ocotp_driver = {
	.name	= "imx_ocotp",
	.probe	= imx_ocotp_probe,
	.of_compatible = DRV_OF_COMPAT(imx_ocotp_dt_ids),
};

static int imx_ocotp_init(void)
{
	platform_driver_register(&imx_ocotp_driver);

	return 0;
}
postcore_initcall(imx_ocotp_init);
