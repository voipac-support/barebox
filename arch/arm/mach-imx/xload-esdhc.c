/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#define pr_fmt(fmt) "xload-esdhc: " fmt

#include <common.h>
#include <io.h>
#include <mci.h>
#include <mach/imx6-regs.h>
#include <mach/xload.h>
#include <linux/sizes.h>
#include "../../../drivers/mci/sdhci.h"
#include "../../../drivers/mci/imx-esdhc.h"

#define SECTOR_SIZE 512

#define esdhc_read32(a)                    readl(a)
#define esdhc_write32(a, v)                writel(v,a)
#define IMX_SDHCI_MIXCTRL  0x48

struct esdhc {
	void __iomem *regs;
	int is_mx6;
};

static void __udelay(int us)
{
	volatile int i;

	for (i = 0; i < us * 4; i++);
}

static u32 esdhc_xfertyp(struct mci_cmd *cmd, struct mci_data *data)
{
	u32 xfertyp = 0;

	if (data)
		xfertyp |= COMMAND_DPSEL | TRANSFER_MODE_MSBSEL |
			TRANSFER_MODE_BCEN |TRANSFER_MODE_DTDSEL;

	if (cmd->resp_type & MMC_RSP_CRC)
		xfertyp |= COMMAND_CCCEN;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		xfertyp |= COMMAND_CICEN;
	if (cmd->resp_type & MMC_RSP_136)
		xfertyp |= COMMAND_RSPTYP_136;
	else if (cmd->resp_type & MMC_RSP_BUSY)
		xfertyp |= COMMAND_RSPTYP_48_BUSY;
	else if (cmd->resp_type & MMC_RSP_PRESENT)
		xfertyp |= COMMAND_RSPTYP_48;

	return COMMAND_CMD(cmd->cmdidx) | xfertyp;
}

static int esdhc_do_data(struct esdhc *esdhc, struct mci_data *data)
{
	void __iomem *regs = esdhc->regs;
	char *buffer;
	u32 databuf;
	u32 size;
	u32 irqstat;
	u32 timeout;
	u32 present;

	buffer = data->dest;

	timeout = 1000000;
	size = data->blocksize * data->blocks;
	irqstat = esdhc_read32(regs + SDHCI_INT_STATUS);

	while (size) {
		present = esdhc_read32(regs + SDHCI_PRESENT_STATE) & PRSSTAT_BREN;
		if (present) {
			databuf = esdhc_read32(regs + SDHCI_BUFFER);
			*((u32 *)buffer) = databuf;
			buffer += 4;
			size -= 4;
		}

		if (!timeout--) {
			pr_err("read time out\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int
esdhc_send_cmd(struct esdhc *esdhc, struct mci_cmd *cmd, struct mci_data *data)
{
	u32	xfertyp, mixctrl;
	u32	irqstat;
	void __iomem *regs = esdhc->regs;
	int ret;
	int timeout;

	esdhc_write32(regs + SDHCI_INT_STATUS, -1);

	/* Wait at least 8 SD clock cycles before the next command */
	__udelay(1);

	if (data) {
		/* Set up for a data transfer if we have one */
		esdhc_write32(regs + SDHCI_DMA_ADDRESS, (u32)data->dest);
		esdhc_write32(regs + SDHCI_BLOCK_SIZE__BLOCK_COUNT, data->blocks << 16 | SECTOR_SIZE);
	}

	/* Figure out the transfer arguments */
	xfertyp = esdhc_xfertyp(cmd, data);

	/* Send the command */
	esdhc_write32(regs + SDHCI_ARGUMENT, cmd->cmdarg);

	if (esdhc->is_mx6) {
		/* write lower-half of xfertyp to mixctrl */
		mixctrl = xfertyp & 0xFFFF;
		/* Keep the bits 22-25 of the register as is */
		mixctrl |= (esdhc_read32(regs + IMX_SDHCI_MIXCTRL) & (0xF << 22));
		esdhc_write32(regs + IMX_SDHCI_MIXCTRL, mixctrl);
	}

	esdhc_write32(regs + SDHCI_TRANSFER_MODE__COMMAND, xfertyp);

	/* Wait for the command to complete */
	timeout = 10000;
	while (!(esdhc_read32(regs + SDHCI_INT_STATUS) & IRQSTAT_CC)) {
		__udelay(1);
		if (!timeout--)
			return -ETIMEDOUT;
	}

	irqstat = esdhc_read32(regs + SDHCI_INT_STATUS);
	esdhc_write32(regs + SDHCI_INT_STATUS, irqstat);

	if (irqstat & CMD_ERR)
		return -EIO;

	if (irqstat & IRQSTAT_CTOE)
		return -ETIMEDOUT;

	/* Copy the response to the response buffer */
	cmd->response[0] = esdhc_read32(regs + SDHCI_RESPONSE_0);

	/* Wait until all of the blocks are transferred */
	if (data) {
		ret = esdhc_do_data(esdhc, data);
		if (ret)
			return ret;
	}

	esdhc_write32(regs + SDHCI_INT_STATUS, -1);

	/* Wait for the bus to be idle */
	timeout = 10000;
	while (esdhc_read32(regs + SDHCI_PRESENT_STATE) &
			(PRSSTAT_CICHB | PRSSTAT_CIDHB | PRSSTAT_DLA)) {
		__udelay(1);
		if (!timeout--)
			return -ETIMEDOUT;
	}

	return 0;
}

static int esdhc_read_blocks(struct esdhc *esdhc, void *dst, size_t len)
{
	struct mci_cmd cmd;
	struct mci_data data;
	u32 val;
	int ret;

	writel(IRQSTATEN_CC | IRQSTATEN_TC | IRQSTATEN_CINT | IRQSTATEN_CTOE |
			IRQSTATEN_CCE | IRQSTATEN_CEBE | IRQSTATEN_CIE |
			IRQSTATEN_DTOE | IRQSTATEN_DCE | IRQSTATEN_DEBE |
			IRQSTATEN_DINT, esdhc->regs + SDHCI_INT_ENABLE);

	val = readl(esdhc->regs + SDHCI_CLOCK_CONTROL__TIMEOUT_CONTROL__SOFTWARE_RESET);
	val |= SYSCTL_HCKEN | SYSCTL_IPGEN;
	writel(val, esdhc->regs + SDHCI_CLOCK_CONTROL__TIMEOUT_CONTROL__SOFTWARE_RESET);

	cmd.cmdidx = MMC_CMD_READ_MULTIPLE_BLOCK;
	cmd.cmdarg = 0;
	cmd.resp_type = MMC_RSP_R1;

	data.dest = dst;
	data.blocks = len / SECTOR_SIZE;
	data.blocksize = SECTOR_SIZE;
	data.flags = MMC_DATA_READ;

	ret = esdhc_send_cmd(esdhc, &cmd, &data);
	if (ret) {
		pr_debug("send command failed with %d\n", ret);
		return ret;
	}

	cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
	cmd.cmdarg = 0;
	cmd.resp_type = MMC_RSP_R1b;

	esdhc_send_cmd(esdhc, &cmd, NULL);

	return 0;
}

int imx6_esdhc_load_image(int instance, void *buf, int len)
{
	struct esdhc esdhc;
	int ret;

	switch (instance) {
	case 0:
		esdhc.regs = IOMEM(MX6_USDHC1_BASE_ADDR);
		break;
	case 1:
		esdhc.regs = IOMEM(MX6_USDHC2_BASE_ADDR);
		break;
	case 2:
		esdhc.regs = IOMEM(MX6_USDHC3_BASE_ADDR);
		break;
	case 3:
		esdhc.regs = IOMEM(MX6_USDHC4_BASE_ADDR);
		break;
	default:
		return -EINVAL;
	}

	esdhc.is_mx6 = 1;

	ret = esdhc_read_blocks(&esdhc, buf, len);
	if (ret)
		return ret;

	return 0;
}

/**
 * imx6_esdhc_start_image - Load and start an image from USDHC controller
 * @instance: The USDHC controller instance (0..4)
 *
 * This uses imx6_esdhc_load_image() to load an image from SD/MMC.
 * It is assumed that the image is the currently running barebox image
 * (This information is used to calculate the length of the image). The
 * image is started afterwards.
 *
 * Return: If successful, this function does not return. A negative error
 * code is returned when this function fails.
 */
int imx6_esdhc_start_image(int instance)
{
	void *buf = (void *)0x10000000;
	u32 *ivt = buf + SZ_1K;
	int ret, len;
	void __noreturn (*bb)(void);
	unsigned int ofs;

	len = imx_image_size();
	len = ALIGN(len, SECTOR_SIZE);

	ret = imx6_esdhc_load_image(instance, buf, 3 * SECTOR_SIZE);
	if (ret)
		return ret;
	if (*(u32 *)(ivt) != 0x402000d1) {
		pr_debug("IVT header not found on SD card. Found 0x%08x instead of 0x402000d1\n",
				*ivt);
		return -EINVAL;
	}

	pr_debug("Check ok, loading image\n");

	ret = imx6_esdhc_load_image(instance, buf, len);
	if (ret) {
		pr_err("Loading image failed with %d\n", ret);
		return ret;
	}

	pr_debug("Image loaded successfully\n");

	ofs = *(ivt + 1) - *(ivt + 8);

	bb = buf + ofs;

	bb();
}
