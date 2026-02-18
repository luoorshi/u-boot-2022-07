/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Allwinner H3 ST7789V GPIO helpers (PIO register access).
 * Pins are read from DTS reset-gpios and dc-gpios (#gpio-cells: bank, pin, flags).
 * pin_index: 0 = reset, 1 = dc.
 */
#ifndef __ST7789V_SUNXI_H3_H
#define __ST7789V_SUNXI_H3_H

struct udevice;

/* Call at bind time so sync path does not use dev_read (avoids hang). */
int st7789v_sunxi_h3_parse_dts(struct udevice *dev);
int st7789v_sunxi_h3_gpio_init(struct udevice *dev);
void st7789v_sunxi_h3_gpio_set_value(int pin_index, int value);

#endif
