/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2021 Nuvoton Technology Corp.
 */

#ifndef __CONFIG_POLEG_H
#define __CONFIG_POLEG_H

#ifndef CONFIG_SYS_L2CACHE_OFF
#define CONFIG_SYS_L2_PL310		1
#define CONFIG_SYS_PL310_BASE	0xF03FC000       /* L2 - Cache Regs Base (4k Space)*/
#endif

#define CONFIG_SYS_MAXARGS              32
#define CONFIG_SYS_CBSIZE               256
#define CONFIG_SYS_PBSIZE               (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_BOOTMAPSZ            (0x30 << 20)
#define CONFIG_SYS_SDRAM_BASE           0x0
#define CONFIG_SYS_INIT_SP_ADDR         (0x00008000 - GENERATED_GBL_DATA_SIZE)

/* Default environemnt variables */
#define CONFIG_SERVERIP                 192.168.0.1
#define CONFIG_IPADDR                   192.168.0.2
#define CONFIG_NETMASK                  255.255.255.0
#define CONFIG_EXTRA_ENV_SETTINGS   "uimage_flash_addr=80200000\0"   \
		"stdin=serial\0"   \
		"stdout=serial\0"   \
		"stderr=serial\0"    \
		"ethact=eth${eth_num}\0"   \
		"romboot=echo Booting from flash; echo +++ uimage at 0x${uimage_flash_addr}; " \
		"echo Using bootargs: ${bootargs};bootm ${uimage_flash_addr}\0" \
		"autostart=yes\0"   \
		"eth_num=0\0"    \
		"ethaddr=00:00:F7:A0:00:FC\0"    \
		"eth1addr=00:00:F7:A0:00:FD\0"   \
		"eth2addr=00:00:F7:A0:00:FE\0"    \
		"eth3addr=00:00:F7:A0:00:FF\0"    \
		"common_bootargs=setenv bootargs earlycon=${earlycon} root=/dev/ram "   \
		"console=${console} mem=${mem} ramdisk_size=48000 basemac=${ethaddr}\0"    \
		"sd_prog=fatload mmc 0 10000000 image-bmc; cp.b 10000000 80000000 ${filesize}\0"  \
		"sd_run=fatload mmc 0 10000000 image-bmc; bootm 10200000\0"   \
		"\0"

#endif
