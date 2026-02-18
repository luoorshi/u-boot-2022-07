// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner H3 (sun8i) PIO register setup for ST7789V reset/dc GPIO.
 * Uses direct register access so display works without DM pinctrl/GPIO
 * (avoids probe-order hangs on sunxi when using gpio_request_by_name).
 *
 * GPIO pins are read from the device tree: reset-gpios and dc-gpios.
 * Sunxi #gpio-cells = 3: (bank, pin, flags). Same CPU, different board
 * wiring only needs DTS change (e.g. reset-gpios = <&pio 0 4 ...> or
 * <&pio 1 3 ...> for PB3).
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/gpio.h>
#include <dm/read.h>
#include <linux/delay.h>

/* Pin index for set_value: 0 = reset, 1 = dc */
#define ST7789V_H3_PIN_RESET  0
#define ST7789V_H3_PIN_DC     1

/* Parsed from DTS: (bank, pin) for reset and dc. Bank 0..8 = PA..PI, 11 = PL (R_PIO). */
static uint reset_bank, reset_pin;
static uint dc_bank, dc_pin;
static bool gpio_parsed;

static void set_pin_value(uint bank, uint pin, int value)
{
	struct sunxi_gpio_reg *regs;
	uint bi;

	if (bank < SUNXI_GPIO_L) {
		regs = (struct sunxi_gpio_reg *)SUNXI_PIO_BASE;
		bi = bank;
	} else {
		regs = (struct sunxi_gpio_reg *)SUNXI_R_PIO_BASE;
		bi = bank - SUNXI_GPIO_L;
	}
	if (value)
		setbits_le32(&regs->gpio_bank[bi].dat, 1u << pin);
	else
		clrbits_le32(&regs->gpio_bank[bi].dat, 1u << pin);
}

/* Convert (bank, pin) from DTS to sunxi global pin number for set_cfgpin/set_pull */
static u32 bank_pin_to_sunxi_pin(uint bank, uint pin)
{
	if (bank < SUNXI_GPIO_L)
		return bank * SUNXI_GPIOS_PER_BANK + pin;
	return SUNXI_GPIO_L_START + pin;
}

/* Parse DTS once; call from probe only (not from bind or sync, to avoid hang). */
int st7789v_sunxi_h3_parse_dts(struct udevice *dev)
{
	struct ofnode_phandle_args args;
	int ret;

	if (gpio_parsed)
		return 0;

	ret = dev_read_phandle_with_args(dev, "reset-gpios", "#gpio-cells", 0, 0, &args);
	if (ret || args.args_count < 2)
		return ret ? ret : -EINVAL;
	reset_bank = args.args[0];
	reset_pin = args.args[1];

	ret = dev_read_phandle_with_args(dev, "dc-gpios", "#gpio-cells", 0, 0, &args);
	if (ret || args.args_count < 2)
		return ret ? ret : -EINVAL;
	dc_bank = args.args[0];
	dc_pin = args.args[1];

	gpio_parsed = true;
	return 0;
}

/* Only configure PIO and set levels; DTS must have been parsed in bind. */
int st7789v_sunxi_h3_gpio_init(struct udevice *dev)
{
	if (!gpio_parsed)
		return -EINVAL;

	/* Mux as GPIO output; same CPU, different pins work via DTS. */
	sunxi_gpio_set_cfgpin(bank_pin_to_sunxi_pin(reset_bank, reset_pin), SUNXI_GPIO_OUTPUT);
	sunxi_gpio_set_cfgpin(bank_pin_to_sunxi_pin(dc_bank, dc_pin), SUNXI_GPIO_OUTPUT);
	sunxi_gpio_set_pull(bank_pin_to_sunxi_pin(reset_bank, reset_pin), SUNXI_GPIO_PULL_DISABLE);
	sunxi_gpio_set_pull(bank_pin_to_sunxi_pin(dc_bank, dc_pin), SUNXI_GPIO_PULL_DISABLE);

	/* Default: reset high (deasserted), dc low (command) */
	st7789v_sunxi_h3_gpio_set_value(ST7789V_H3_PIN_RESET, 1);
	st7789v_sunxi_h3_gpio_set_value(ST7789V_H3_PIN_DC, 0);
	return 0;
}

void st7789v_sunxi_h3_gpio_set_value(int pin_index, int value)
{
	if (!gpio_parsed)
		return;
	if (pin_index == ST7789V_H3_PIN_RESET)
		set_pin_value(reset_bank, reset_pin, value);
	else if (pin_index == ST7789V_H3_PIN_DC)
		set_pin_value(dc_bank, dc_pin, value);
}
