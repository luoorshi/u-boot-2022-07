// SPDX-License-Identifier: GPL-2.0+
/*
 * Generic U-Boot driver for the ST7789V LCD Controller (SPI)
 *
 * 240x320 RGB565, D/C and RESET via GPIO. SoC-agnostic; works on any
 * platform with DM_SPI and DM_GPIO.
 *
 * Based on kernel driver drivers/staging/fbtft/fb_st7789v.c
 * Copyright (C) 2015 Dennis Menschel
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <spi.h>
#include <video.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#if CONFIG_IS_ENABLED(VIDEO_ST7789V_SUNXI_H3)
#include "st7789v/st7789v_sunxi_h3.h"
#endif

#define ST7789V_WIDTH		135
#define ST7789V_HEIGHT		240
#define ST7789V_BPP		16

#define ST7789V_RESET_DELAY_MS		10
#define ST7789V_RESET_HOLD_MS		120
#define ST7789V_SLEEP_EXIT_DELAY_MS	100

#define ST7789V_FB_SUM_BLOCK_SIZE	4

#define MADCTL_BGR	BIT(3)
#define MADCTL_MV	BIT(5)
#define MADCTL_MX	BIT(6)
#define MADCTL_MY	BIT(7)

#define MIPI_DCS_EXIT_SLEEP_MODE	0x11
#define MIPI_DCS_SET_DISPLAY_ON		0x29
#define MIPI_DCS_SET_PIXEL_FORMAT	0x3A
#define MIPI_DCS_PIXEL_FMT_16BIT		0x55
#define MIPI_DCS_SET_ADDRESS_MODE	0x36
#define MIPI_DCS_SET_COLUMN_ADDRESS	0x2A
#define MIPI_DCS_SET_PAGE_ADDRESS	0x2B
#define MIPI_DCS_MEMORY_WRITE		0x2C

#define ST7789V_PORCTRL		0xB2
#define ST7789V_GCTRL		0xB7
#define ST7789V_VCOMS		0xBB
#define ST7789V_VDVVRHEN	0xC2
#define ST7789V_VRHS		0xC3
#define ST7789V_VDVS		0xC4
#define ST7789V_VCMOFSET	0xC5
#define ST7789V_VMCTRL1		0xC0
#define ST7789V_FRCTRL2		0xC6
#define ST7789V_PWCTRL1		0xD0
#define ST7789V_PVGAMCTRL	0xE0
#define ST7789V_NVGAMCTRL	0xE1
#define ST7789V_PVGAMCTRL_PARAM	0xE8
#define ST7789V_NVGAMCTRL_PARAM	0xE9

struct st7789v_init_cmd {
	u8 cmd;
	const u8 *data;
	u8 data_len;
	u16 delay_ms;
};

struct st7789v_priv {
	struct gpio_desc reset_gpio;
	struct gpio_desc dc_gpio;
	struct udevice *dev;
	bool gpio_requested;
	bool inited;
	u16 col_offset;
	u16 row_offset;
	bool bgr;
};

static const u8 init_data_pixfmt[] = {0x05};
static const u8 init_data_vcmofset[] = {0x20};
static const u8 init_data_madctl[] = {0x00};
static const u8 init_data_porctrl[] = {0x05, 0x05, 0x00, 0x33, 0x33};
static const u8 init_data_gctrl[] = {0x75};
static const u8 init_data_vcoms[] = {0x22};
static const u8 init_data_vmctrl1[] = {0x2C};
static const u8 init_data_vdvvrhen[] = {0x01, 0xFF};
static const u8 init_data_vrhs[] = {0x13};
static const u8 init_data_vdvs[] = {0x20};
static const u8 init_data_frctrl2[] = {0x01};
static const u8 init_data_pwctrl1[] = {0xA4, 0xA1};
static const u8 init_data_pvgam_param[] = {0x03};
static const u8 init_data_nvgam_param[] = {0x09, 0x09, 0x08};
static const u8 init_data_pvgam[] = {0xD0, 0x05, 0x0A, 0x09, 0x08, 0x05, 0x2E, 0x44, 0x45, 0x0F, 0x17, 0x16, 0x2B, 0x33};
static const u8 init_data_nvgam[] = {0xD0, 0x05, 0x0A, 0x09, 0x08, 0x05, 0x2E, 0x43, 0x45, 0x0F, 0x16, 0x16, 0x2B, 0x33};

static const struct st7789v_init_cmd st7789v_init_cmds[] = {
	{MIPI_DCS_SET_PIXEL_FORMAT, init_data_pixfmt, 1, 0},
	{ST7789V_VCMOFSET, init_data_vcmofset, 1, 0},
	{MIPI_DCS_SET_ADDRESS_MODE, init_data_madctl, 1, 0},
	{ST7789V_PORCTRL, init_data_porctrl, 5, 0},
	{ST7789V_GCTRL, init_data_gctrl, 1, 0},
	{ST7789V_VCOMS, init_data_vcoms, 1, 0},
	{ST7789V_VMCTRL1, init_data_vmctrl1, 1, 0},
	{ST7789V_VDVVRHEN, init_data_vdvvrhen, 2, 0},
	{ST7789V_VRHS, init_data_vrhs, 1, 0},
	{ST7789V_VDVS, init_data_vdvs, 1, 0},
	{ST7789V_FRCTRL2, init_data_frctrl2, 1, 0},
	{ST7789V_PWCTRL1, init_data_pwctrl1, 2, 0},
	{ST7789V_PVGAMCTRL_PARAM, init_data_pvgam_param, 1, 0},
	{ST7789V_NVGAMCTRL_PARAM, init_data_nvgam_param, 3, 0},
	{ST7789V_PVGAMCTRL, init_data_pvgam, 14, 0},
	{ST7789V_NVGAMCTRL, init_data_nvgam, 14, 0},
	{MIPI_DCS_SET_DISPLAY_ON, NULL, 0, 20},
};

static int st7789v_set_dc(struct udevice *dev, int value);

static void st7789v_set_reset(struct udevice *dev, int value)
{
	if (IS_ENABLED(CONFIG_VIDEO_ST7789V_SUNXI_H3))
		st7789v_sunxi_h3_gpio_set_value(0, value);
	else
		dm_gpio_set_value(&((struct st7789v_priv *)dev_get_priv(dev))->reset_gpio, value);
}

static int st7789v_set_dc(struct udevice *dev, int value)
{
	struct st7789v_priv *priv = dev_get_priv(dev);

	if (IS_ENABLED(CONFIG_VIDEO_ST7789V_SUNXI_H3)) {
		st7789v_sunxi_h3_gpio_set_value(1, value);
		return 0;
	}
	return dm_gpio_set_value(&priv->dc_gpio, value);
}

static int st7789v_write_cmd(struct udevice *dev, u8 cmd)
{
	struct st7789v_priv *priv = dev_get_priv(dev);
	int ret;

	if (!priv)
		return -EINVAL;

	ret = st7789v_set_dc(dev, 0);
	if (ret)
		return ret;

	return dm_spi_xfer(dev, 8, &cmd, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
}

static int st7789v_write_data(struct udevice *dev, const u8 *data, size_t len)
{
	struct st7789v_priv *priv = dev_get_priv(dev);
	int ret;

	if (!priv)
		return -EINVAL;

	if (!len)
		return 0;

	ret = st7789v_set_dc(dev, 1);
	if (ret)
		return ret;

	return dm_spi_xfer(dev, len * 8, (void *)data, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
}

static int st7789v_write_cmd_param(struct udevice *dev, u8 cmd, const u8 *params, size_t num)
{
	int ret;

	ret = st7789v_write_cmd(dev, cmd);
	if (ret)
		return ret;

	if (params && num)
		ret = st7789v_write_data(dev, params, num);

	return ret;
}

static int st7789v_hw_reset(struct udevice *dev)
{
	st7789v_set_reset(dev, 1);
	mdelay(ST7789V_RESET_DELAY_MS);
	st7789v_set_reset(dev, 0);
	mdelay(ST7789V_RESET_DELAY_MS);
	st7789v_set_reset(dev, 1);
	mdelay(ST7789V_RESET_HOLD_MS);
	return 0;
}

static int st7789v_send_init_sequence(struct udevice *dev)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(st7789v_init_cmds); i++) {
		const struct st7789v_init_cmd *cmd = &st7789v_init_cmds[i];

		if (cmd->data_len > 0)
			ret = st7789v_write_cmd_param(dev, cmd->cmd, cmd->data, cmd->data_len);
		else
			ret = st7789v_write_cmd(dev, cmd->cmd);

		if (ret)
			return ret;

		if (cmd->delay_ms > 0)
			mdelay(cmd->delay_ms);
	}
	return 0;
}

static int st7789v_init_display(struct udevice *dev)
{
	int ret;

	st7789v_hw_reset(dev);

	ret = st7789v_write_cmd(dev, MIPI_DCS_EXIT_SLEEP_MODE);
	if (ret)
		return ret;
	mdelay(ST7789V_SLEEP_EXIT_DELAY_MS);

	return st7789v_send_init_sequence(dev);
}

static int st7789v_clear_screen(struct udevice *dev)
{
	struct st7789v_priv *priv = dev_get_priv(dev);
	u8 col_buf[4], row_buf[4];
	u16 *black_buffer;
	int ret, i;
	const u16 black_pixel = 0x0000;

	col_buf[0] = priv->col_offset >> 8;
	col_buf[1] = priv->col_offset & 0xFF;
	col_buf[2] = (priv->col_offset + ST7789V_WIDTH - 1) >> 8;
	col_buf[3] = (priv->col_offset + ST7789V_WIDTH - 1) & 0xFF;

	row_buf[0] = priv->row_offset >> 8;
	row_buf[1] = priv->row_offset & 0xFF;
	row_buf[2] = (priv->row_offset + ST7789V_HEIGHT - 1) >> 8;
	row_buf[3] = (priv->row_offset + ST7789V_HEIGHT - 1) & 0xFF;

	ret = st7789v_write_cmd(dev, MIPI_DCS_SET_COLUMN_ADDRESS);
	if (ret)
		return ret;
	ret = st7789v_write_data(dev, col_buf, 4);
	if (ret)
		return ret;

	ret = st7789v_write_cmd(dev, MIPI_DCS_SET_PAGE_ADDRESS);
	if (ret)
		return ret;
	ret = st7789v_write_data(dev, row_buf, 4);
	if (ret)
		return ret;

	ret = st7789v_write_cmd(dev, MIPI_DCS_MEMORY_WRITE);
	if (ret)
		return ret;

	black_buffer = malloc(ST7789V_WIDTH * sizeof(u16));
	if (!black_buffer)
		return -ENOMEM;

	for (i = 0; i < ST7789V_WIDTH; i++)
		black_buffer[i] = black_pixel;

	for (i = 0; i < ST7789V_HEIGHT; i++) {
		ret = st7789v_set_dc(dev, 1);
		if (ret) {
			free(black_buffer);
			return ret;
		}
		ret = dm_spi_xfer(dev, ST7789V_WIDTH * 16, black_buffer, NULL,
				  SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret) {
			free(black_buffer);
			return ret;
		}
	}

	free(black_buffer);
	return 0;
}

static int st7789v_set_madctl(struct udevice *dev, int rotation)
{
	struct st7789v_priv *priv = dev_get_priv(dev);
	u8 madctl = 0;

	switch (rotation) {
	case 90:
		madctl |= MADCTL_MV | MADCTL_MY;
		break;
	case 180:
		madctl |= MADCTL_MX | MADCTL_MY;
		break;
	case 270:
		madctl |= MADCTL_MV | MADCTL_MX;
		break;
	default:
		break;
	}

	if (priv && priv->bgr)
		madctl |= MADCTL_BGR;

	printf("Lois_debug: st7789v_set_madctl madctl=0x%02X\n", madctl);
	return st7789v_write_cmd_param(dev, MIPI_DCS_SET_ADDRESS_MODE, &madctl, 1);
}

static u32 st7789v_calc_fb_checksum(struct video_priv *uc_priv)
{
	ulong size = (ulong)uc_priv->line_length * uc_priv->ysize;
	ulong i;
	u32 sum = 0;
	const u8 *fb = (const u8 *)uc_priv->fb;

	for (i = 0; i + ST7789V_FB_SUM_BLOCK_SIZE <= size; i += ST7789V_FB_SUM_BLOCK_SIZE)
		sum += *(const u32 *)(fb + i);
	if (size & 2)
		sum += *(const u16 *)(fb + (size & ~3));
	if (size & 1)
		sum += fb[size - 1];
	return sum;
}

static int st7789v_request_gpios(struct udevice *dev)
{
	struct st7789v_priv *priv = dev_get_priv(dev);
	int ret;

#if CONFIG_IS_ENABLED(VIDEO_ST7789V_SUNXI_H3)
	ret = st7789v_sunxi_h3_gpio_init(dev);
	if (ret)
		return ret;
#else
	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset_gpio, GPIOD_IS_OUT);
	if (ret)
		return ret;

	ret = gpio_request_by_name(dev, "dc-gpios", 0, &priv->dc_gpio, GPIOD_IS_OUT);
	if (ret)
		return ret;
#endif

	priv->gpio_requested = true;
	return 0;
}

static int st7789v_ensure_parent_probed(struct udevice *dev)
{
	if (dev->parent && !device_active(dev->parent))
		return device_probe(dev->parent);
	return 0;
}

static int st7789v_deferred_init(struct udevice *vid)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(vid);
	struct st7789v_priv *priv = dev_get_priv(vid);
	struct udevice *dev = priv->dev;
	int rot_deg = (int)uc_priv->rot * 90;
	int ret;

	ret = st7789v_ensure_parent_probed(dev);
	if (ret)
		return ret;

	if (!priv->gpio_requested) {
		ret = st7789v_request_gpios(dev);
		if (ret)
			return ret;
	}

	ret = dm_spi_claim_bus(dev);
	if (ret)
		return ret;

	ret = st7789v_init_display(dev);
	if (ret) {
		dm_spi_release_bus(dev);
		return ret;
	}

	ret = st7789v_set_madctl(dev, rot_deg);
	if (ret) {
		dm_spi_release_bus(dev);
		return ret;
	}

	ret = st7789v_clear_screen(dev);
	dm_spi_release_bus(dev);
	if (ret)
		return ret;

	priv->inited = true;
	return 0;
}

static int st7789v_sync(struct udevice *vid)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(vid);
	struct st7789v_priv *priv = dev_get_priv(vid);
	struct udevice *dev;
	u8 col_buf[4], row_buf[4];
	size_t line_bytes;
	int ret;
	unsigned int y;
	static u32 last_fb_sum;


	if (!uc_priv || !priv || !priv->dev)
		return -EINVAL;

	dev = priv->dev;
	line_bytes = (size_t)uc_priv->xsize * 2;

	if (!priv->inited) {
		ret = st7789v_deferred_init(vid);
		if (ret)
			return ret;
	}

	if (st7789v_calc_fb_checksum(uc_priv) == last_fb_sum)
		return 0;
	last_fb_sum = st7789v_calc_fb_checksum(uc_priv);

	ret = dm_spi_claim_bus(dev);
	if (ret) {
		dev_err(dev, "Failed to claim SPI bus: %d\n", ret);
		return ret;
	}

	col_buf[0] = priv->col_offset >> 8;
	col_buf[1] = priv->col_offset & 0xFF;
	col_buf[2] = (priv->col_offset + uc_priv->xsize - 1) >> 8;
	col_buf[3] = (priv->col_offset + uc_priv->xsize - 1) & 0xFF;
	st7789v_write_cmd(dev, MIPI_DCS_SET_COLUMN_ADDRESS);
	st7789v_write_data(dev, col_buf, 4);

	row_buf[0] = priv->row_offset >> 8;
	row_buf[1] = priv->row_offset & 0xFF;
	row_buf[2] = (priv->row_offset + uc_priv->ysize - 1) >> 8;
	row_buf[3] = (priv->row_offset + uc_priv->ysize - 1) & 0xFF;
	st7789v_write_cmd(dev, MIPI_DCS_SET_PAGE_ADDRESS);
	st7789v_write_data(dev, row_buf, 4);

	st7789v_write_cmd(dev, MIPI_DCS_MEMORY_WRITE);

	for (y = 0; y < uc_priv->ysize; y++) {
		ret = st7789v_set_dc(dev, 1);
		if (ret)
			goto release;
		ret = dm_spi_xfer(dev, line_bytes * 8,
				  (void *)(uc_priv->fb + y * uc_priv->line_length),
				  NULL, SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret)
			goto release;
	}

release:
	dm_spi_release_bus(dev);
	return ret;
}

static int st7789v_parse_rotation(struct udevice *dev)
{
	int rot = dev_read_u32_default(dev, "rotation", 0);

	switch (rot) {
	case 90:
		return 1;
	case 180:
		return 2;
	case 270:
		return 3;
	default:
		return 0;
	}
}

static int st7789v_probe(struct udevice *dev)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct st7789v_priv *priv = dev_get_priv(dev);
	int ret;

	if (!dev || !priv || !uc_priv)
		return -EINVAL;

	priv->gpio_requested = false;
	priv->inited = false;
	priv->dev = dev;

	uc_priv->bpix = VIDEO_BPP16;
	uc_priv->xsize = ST7789V_WIDTH;
	uc_priv->ysize = ST7789V_HEIGHT;
	uc_priv->rot = st7789v_parse_rotation(dev);

	priv->col_offset = dev_read_u32_default(dev, "col-offset", 0);
	priv->row_offset = dev_read_u32_default(dev, "row-offset", 0);
	priv->bgr = dev_read_bool(dev, "bgr");

#if CONFIG_IS_ENABLED(VIDEO_ST7789V_SUNXI_H3)
	ret = st7789v_sunxi_h3_parse_dts(dev);
	if (ret)
		return ret;
#endif

	return 0;
}

static int st7789v_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	plat->size = (ulong)ST7789V_WIDTH * ST7789V_HEIGHT * (ST7789V_BPP / 8);
	return 0;
}

static const struct video_ops st7789v_ops = {
	.video_sync = st7789v_sync,
};

static const struct udevice_id st7789v_ids[] = {
	{ .compatible = "sitronix,st7789v" },
	{ }
};

U_BOOT_DRIVER(st7789v_video) = {
	.name   = "st7789v_video",
	.id     = UCLASS_VIDEO,
	.of_match = st7789v_ids,
	.ops    = &st7789v_ops,
	.plat_auto = sizeof(struct video_uc_plat),
	.bind   = st7789v_bind,
	.probe  = st7789v_probe,
	.priv_auto = sizeof(struct st7789v_priv),
	.flags  = DM_FLAG_PRE_RELOC,
};
