/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * siemens pxm2
 * (C) Copyright 2013 Siemens Schweiz AG
 * (C) Heiko Schocher, DENX Software Engineering, hs@denx.de.
 *
 * Based on:
 * U-Boot file:/include/configs/am335x_evm.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef __CONFIG_PXM2_H
#define __CONFIG_PXM2_H

#include "siemens-am33x-common.h"

#define DDR_IOCTRL_VAL		0x18b
#define DDR_PLL_FREQ		266

#define BOARD_DFU_BUTTON_GPIO	59
#define BOARD_LCD_POWER		111
#define BOARD_BACK_LIGHT	112
#define BOARD_TOUCH_POWER	57

#define CONFIG_ENV_SETTINGS_BUTTONS_AND_LEDS \
	"button_dfu0=59\0" \
	"led0=117,0,1\0" \

 /* Physical Memory Map */
#define CONFIG_MAX_RAM_BANK_SIZE	(512 << 20)	/* 1GB */

#define CONFIG_FACTORYSET

#ifndef CONFIG_SPL_BUILD

/* Use common default */

/* Default env settings */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"hostname=pxm2\0" \
	"ubi_off=2048\0"\
	"nand_img_size=0x500000\0" \
	"optargs=\0" \
	"preboot=draco_led 0\0" \
	CONFIG_ENV_SETTINGS_BUTTONS_AND_LEDS \
	"splashpos=m,m\0"	\
	CONFIG_ENV_SETTINGS_V1 \
	CONFIG_ENV_SETTINGS_NAND_V1 \
	"mmc_dev=0\0" \
	"mmc_root=/dev/mmcblk0p2 rw\0" \
	"mmc_root_fs_type=ext4 rootwait\0" \
	"mmc_load_uimage=" \
		"mmc rescan; " \
		"setenv bootfile uImage;" \
		"fatload mmc ${mmc_dev} ${kloadaddr} ${bootfile}\0" \
	"loadbootenv=fatload mmc ${mmc_dev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t $loadaddr $filesize\0" \
	"mmc_args=run bootargs_defaults;" \
		"mtdparts default;" \
		"setenv bootargs ${bootargs} " \
		"root=${mmc_root} ${mtdparts}" \
		"rootfstype=${mmc_root_fs_type} ip=${ip_method} " \
		"eth=${ethaddr} " \
		"\0" \
	"mmc_boot=run mmc_args; " \
		"run mmc_load_uimage; " \
		"bootm ${kloadaddr}\0" \
	""

#ifndef CONFIG_RESTORE_FLASH
/* set to negative value for no autoboot */
#endif
#endif	/* CONFIG_SPL_BUILD */

#endif	/* ! __CONFIG_PXM2_H */