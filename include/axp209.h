/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 */

#include <linux/bitops.h>

enum axp209_reg {
	AXP209_POWER_STATUS = 0x00,
	AXP209_CHIP_VERSION = 0x03,
	AXP209_OUTPUT_CTRL = 0x12,
	AXP209_DCDC2_VOLTAGE = 0x23,
	AXP209_VRC_DCDC2_LDO3 = 0x25,
	AXP209_DCDC3_VOLTAGE = 0x27,
	AXP209_LDO24_VOLTAGE = 0x28,
	AXP209_LDO3_VOLTAGE = 0x29,
	AXP209_IRQ_ENABLE1 = 0x40,
	AXP209_IRQ_ENABLE2 = 0x41,
	AXP209_IRQ_ENABLE3 = 0x42,
	AXP209_IRQ_ENABLE4 = 0x43,
	AXP209_IRQ_ENABLE5 = 0x44,
	AXP209_IRQ_STATUS5 = 0x4c,
	AXP209_SHUTDOWN = 0x32,
};

#define AXP209_POWER_STATUS_ON_BY_DC	BIT(0)
#define AXP209_POWER_STATUS_VBUS_USABLE	BIT(4)

#define AXP209_CHIP_VERSION_MASK	0x0f

#define AXP209_OUTPUT_CTRL_EXTEN	BIT(0)
#define AXP209_OUTPUT_CTRL_DCDC3	BIT(1)
#define AXP209_OUTPUT_CTRL_LDO2		BIT(2)
#define AXP209_OUTPUT_CTRL_LDO4		BIT(3)
#define AXP209_OUTPUT_CTRL_DCDC2	BIT(4)
#define AXP209_OUTPUT_CTRL_LDO3		BIT(6)

/*
 * AXP209 datasheet contains wrong information about LDO3 VRC:
 * - VRC is actually enabled when BIT(1) is True
 * - VRC is actually not enabled by default (BIT(3) = 0 after reset)
 */
#define AXP209_VRC_LDO3_EN		BIT(3)
#define AXP209_VRC_DCDC2_EN		BIT(2)
#define AXP209_VRC_LDO3_800uV_uS	(BIT(1) | AXP209_VRC_LDO3_EN)
#define AXP209_VRC_LDO3_1600uV_uS	AXP209_VRC_LDO3_EN
#define AXP209_VRC_DCDC2_800uV_uS	(BIT(0) | AXP209_VRC_DCDC2_EN)
#define AXP209_VRC_DCDC2_1600uV_uS	AXP209_VRC_DCDC2_EN
#define AXP209_VRC_LDO3_MASK		0xa
#define AXP209_VRC_DCDC2_MASK		0x5
#define AXP209_VRC_DCDC2_SLOPE_SET(reg, cfg) \
	(((reg) & ~AXP209_VRC_DCDC2_MASK) | \
	((cfg) & AXP209_VRC_DCDC2_MASK))
#define AXP209_VRC_LDO3_SLOPE_SET(reg, cfg) \
	(((reg) & ~AXP209_VRC_LDO3_MASK) | \
	((cfg) & AXP209_VRC_LDO3_MASK))

#define AXP209_LDO24_LDO2_MASK		0xf0
#define AXP209_LDO24_LDO4_MASK		0x0f
#define AXP209_LDO24_LDO2_SET(reg, cfg)	\
	(((reg) & ~AXP209_LDO24_LDO2_MASK) | \
	(((cfg) << 4) & AXP209_LDO24_LDO2_MASK))
#define AXP209_LDO24_LDO4_SET(reg, cfg)	\
	(((reg) & ~AXP209_LDO24_LDO4_MASK) | \
	(((cfg) << 0) & AXP209_LDO24_LDO4_MASK))

#define AXP209_LDO3_VOLTAGE_FROM_LDO3IN	BIT(7)
#define AXP209_LDO3_VOLTAGE_MASK	0x7f
#define AXP209_LDO3_VOLTAGE_SET(x)	((x) & AXP209_LDO3_VOLTAGE_MASK)

#define AXP209_IRQ5_PEK_UP		BIT(6)
#define AXP209_IRQ5_PEK_DOWN		BIT(5)

#define AXP209_POWEROFF			BIT(7)

/* For axp_gpio.c */
#ifdef CONFIG_AXP209_POWER
#define AXP_POWER_STATUS		0x00
#define AXP_POWER_STATUS_ALDO_IN		BIT(0)
#define AXP_POWER_STATUS_VBUS_PRESENT		BIT(5)
#define AXP_GPIO0_CTRL			0x90
#define AXP_GPIO1_CTRL			0x92
#define AXP_GPIO2_CTRL			0x93
#define AXP_GPIO_CTRL_OUTPUT_LOW	0x00 /* Drive pin low */
#define AXP_GPIO_CTRL_OUTPUT_HIGH	0x01 /* Drive pin high */
#define AXP_GPIO_CTRL_INPUT		0x02 /* Input */
#define AXP_GPIO_STATE			0x94
#define AXP_GPIO_STATE_OFFSET		4
#endif
