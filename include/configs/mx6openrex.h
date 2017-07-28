/*
 * Copyright (C) 2015-2017 Voipac.
 *
 * Configuration settings for the Fedevel i.MX6 OpenREX board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>
#include "mx6rex_common.h"

#define CONFIG_MACH_TYPE                    5082

#define CONFIG_BOARD_DEFAULT_ARCH_PREFIX    CONFIG_MODULE_TYPE_PREFIX
#define CONFIG_BOARD_DEFAULT_ARCH_POSTFIX   CONFIG_MODULE_TYPE_POSTFIX

#if defined(CONFIG_DDR_SIZE)
#define CONFIG_BOARD_PHYS_SDRAM_SIZE        CONFIG_DDR_SIZE
#else
#define CONFIG_BOARD_PHYS_SDRAM_SIZE        SZ_512M
#warning "Using default SDRAM size"
#endif

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR           USDHC2_BASE_ADDR
#define CONFIG_SYS_FSL_USDHC_NUM            1

/* SPI Configs */
#define CONFIG_SF_DEFAULT_BUS               2
#define CONFIG_SF_DEFAULT_CS                2

/* PCI Configs */
#define CONFIG_PCIE_IMX_PERST_GPIO          IMX_GPIO_NR(4, 15)

/* Miscellaneous configurable options */
#define CONFIG_SYS_PROMPT                   "OpenRex U-Boot > "
#define CONFIG_SYS_PROMPT_HUSH_PS2          "OpenRex U-Boot > "

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE                   (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

/* Physical Memory Map */
#define PHYS_SDRAM_SIZE                     CONFIG_BOARD_PHYS_SDRAM_SIZE

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_SIZE                     (SZ_8K)
#define CONFIG_ENV_OFFSET                   (8 * SZ_64K)
#define CONFIG_SYS_MMC_ENV_DEV              0 /* SDHC2 */
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_SIZE                     (SZ_8K)
#define CONFIG_ENV_OFFSET                   (8 * SZ_64K)
#define CONFIG_ENV_SECT_SIZE                SZ_64K
#define CONFIG_ENV_SPI_BUS                  CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS                   CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MAX_HZ               CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_SPI_MODE                 CONFIG_SF_DEFAULT_MODE
#elif defined(CONFIG_ENV_IS_IN_SATA)
#define CONFIG_ENV_SIZE                     (SZ_8K)
#define CONFIG_ENV_OFFSET                   (12 * SZ_64K)
#define CONFIG_SATA_ENV_DEV                 0
#define CONFIG_SYS_DCACHE_OFF               /* remove when sata driver support cache */
#endif

/* PMIC Configs */
#define CONFIG_POWER
#ifdef CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR      0x08
#endif

/* Env settings */
#define CONFIG_ENV_CONSOLE_DEV              "ttymxc0"
#define CONFIG_ENV_MMCROOT                  "/dev/mmcblk1p2"
#define CONFIG_ENV_DEFAULT_UBT_FILE         "u-boot-" CONFIG_BOARD_DEFAULT_ARCH_PREFIX "-openrex" CONFIG_BOARD_DEFAULT_ARCH_POSTFIX ".imx"
#define CONFIG_ENV_DEFAULT_IMG_FILE         "zImage-" CONFIG_BOARD_DEFAULT_ARCH_PREFIX "-openrex" CONFIG_BOARD_DEFAULT_ARCH_POSTFIX
#define CONFIG_ENV_DEFAULT_FDT_FILE                   CONFIG_BOARD_DEFAULT_ARCH_PREFIX "-openrex" CONFIG_BOARD_DEFAULT_ARCH_POSTFIX ".dtb"
#define CONFIG_ENV_DEFAULT_SCR_FILE         "boot-"   CONFIG_BOARD_DEFAULT_ARCH_PREFIX "-openrex" CONFIG_BOARD_DEFAULT_ARCH_POSTFIX ".scr"
#define CONFIG_ENV_DEFAULT_ETH_ADDR         "00:0D:15:00:D1:75"
#define CONFIG_ENV_DEFAULT_CLIENT_IP        "192.168.0.150"
#define CONFIG_ENV_DEFAULT_SERVER_IP        "192.168.0.1"
#define CONFIG_ENV_DEFAULT_NETMASK          "255.255.255.0"
#define CONFIG_ENV_DEFAULT_TFTP_DIR         "imx6"
#ifdef CONFIG_SYS_MMC_ENV_DEV
#define CONFIG_ENV_MMC_ENV_DEV              __stringify(CONFIG_SYS_MMC_ENV_DEV)
#else
#define CONFIG_ENV_MMC_ENV_DEV              __stringify(0)
#endif

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
	"rdinit=/linuxrc " \
	"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
	"g_mass_storage.file=/fat g_mass_storage.ro=1 " \
	"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF " \
	"g_mass_storage.iSerialNumber=\"\" " \
	"enable_wait_mode=off\0" \
	"initrd_addr=0x12C00000\0" \
	"initrd_high=0xffffffff\0" \
	"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0"

#if defined(CONFIG_ANDROID_SUPPORT)

#define CONFIG_EXTRA_ENV_SETTINGS \
    "splashpos=m,m\0"             \
    "fdt_high=0xffffffff\0"       \
    "initrd_high=0xffffffff\0"    \

#else

#define CONFIG_UPDATE_ENV_SETTINGS \
	"set_ethernet=" \
		"if test -n ${ethaddr}; then; else " \
			"setenv ethaddr  " CONFIG_ENV_DEFAULT_ETH_ADDR  "; " \
		"fi; " \
		"if test -n ${ipaddr}; then; else " \
			"setenv ipaddr   " CONFIG_ENV_DEFAULT_CLIENT_IP "; " \
		"fi; " \
		"if test -n ${serverip}; then; else " \
			"setenv serverip " CONFIG_ENV_DEFAULT_SERVER_IP "; " \
		"fi; " \
		"if test -n ${netmask}; then; else " \
			"setenv netmask  " CONFIG_ENV_DEFAULT_NETMASK   "; " \
		"fi\0" \
	"update_set_filename=" \
		"if test -n ${upd_uboot}; then; else " \
			"setenv upd_uboot " CONFIG_ENV_DEFAULT_UBT_FILE  "; " \
		"fi; " \
		"if test -n ${upd_kernel}; then; else " \
			"setenv upd_kernel " CONFIG_ENV_DEFAULT_IMG_FILE "; " \
		"fi; " \
		"if test -n ${upd_fdt}; then; else " \
			"setenv upd_fdt    " CONFIG_ENV_DEFAULT_FDT_FILE "; " \
		"fi; " \
		"if test -n ${upd_script}; then; else " \
			"setenv upd_script " CONFIG_ENV_DEFAULT_SCR_FILE "; " \
		"fi\0"
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_UPDATE_ENV_SETTINGS_UBOOT \
	"update_uboot=" \
		"run set_ethernet; " \
		"run update_set_filename; " \
		"if mmc dev ${mmcdev}; then "	\
			"if tftp ${tftp_dir}/${upd_uboot}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; " \
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; " \
		"fi\0"
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_UPDATE_ENV_SETTINGS_UBOOT \
	"update_uboot=" \
		"run set_ethernet; " \
		"run update_set_filename; " \
		"if sf probe ${spidev}:${spics}; then "	\
			"if tftp ${tftp_dir}/${upd_uboot}; then " \
				"setexpr align_sz ${filesize} + 0x7FFF; " \
				"setexpr align_sz ${align_sz} / 0x8000; " \
				"setexpr align_sz ${align_sz} * 0x8000; " \
				"sf erase 0x0 ${align_sz}; " \
				"sf write ${loadaddr} 0x400 ${filesize}; " \
			"fi; " \
		"fi\0"
#elif defined(CONFIG_ENV_IS_IN_SATA)
#define CONFIG_UPDATE_ENV_SETTINGS_UBOOT \
	"update_uboot=" \
		"run set_ethernet; " \
		"run update_set_filename; " \
		"if sata init; then " \
			"if tftp ${tftp_dir}/${upd_uboot}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; " \
				"sata write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; " \
		"fi\0"
#else
#define CONFIG_UPDATE_ENV_SETTINGS_UBOOT \
	"update_uboot=\0"
#endif
#define CONFIG_UPDATE_ENV_SETTINGS_KERNEL \
	"update_kernel=" \
		"run set_ethernet; " \
		"run update_set_filename; " \
		"if mmc dev ${mmcdev}; then "	\
			"if tftp ${tftp_dir}/${upd_kernel}; then " \
				"fatwrite mmc ${mmcdev}:${mmcpart} " \
				"${loadaddr} ${image} ${filesize}; " \
			"fi; " \
		"fi\0"
#define CONFIG_UPDATE_ENV_SETTINGS_FDT \
	"update_fdt=" \
		"run set_ethernet; " \
		"run update_set_filename; " \
		"if mmc dev ${mmcdev}; then "	\
			"if tftp ${tftp_dir}/${upd_fdt}; then " \
				"fatwrite mmc ${mmcdev}:${mmcpart} " \
				"${loadaddr} ${fdt_file} ${filesize}; " \
			"fi; " \
		"fi\0"
#define CONFIG_UPDATE_ENV_SETTINGS_SCRIPT \
	"update_script=" \
		"run set_ethernet; " \
		"run update_set_filename; " \
		"if mmc dev ${mmcdev}; then "	\
			"if tftp ${tftp_dir}/${upd_script}; then " \
				"fatwrite mmc ${mmcdev}:${mmcpart} " \
				"${loadaddr} ${script} ${filesize}; " \
			"fi; " \
		"fi\0"

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"tftp_dir=" CONFIG_ENV_DEFAULT_TFTP_DIR "\0" \
	"uboot="    CONFIG_ENV_DEFAULT_UBT_FILE "\0" \
	"image="    "zImage" "\0" \
	"fdt_file=" CONFIG_ENV_DEFAULT_FDT_FILE "\0" \
	"script="   CONFIG_ENV_DEFAULT_SCR_FILE "\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"console=" CONFIG_ENV_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"smp=" CONFIG_SYS_NOSMP "\0"\
	"mmcdev=" CONFIG_ENV_MMC_ENV_DEV "\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_ENV_MMCROOT " rootwait rw\0" \
	"spidev=" __stringify(CONFIG_ENV_SPI_BUS) "\0" \
	"spics=" __stringify(CONFIG_ENV_SPI_CS) "\0" \
	CONFIG_UPDATE_ENV_SETTINGS \
	CONFIG_UPDATE_ENV_SETTINGS_UBOOT \
	CONFIG_UPDATE_ENV_SETTINGS_KERNEL \
	CONFIG_UPDATE_ENV_SETTINGS_FDT \
	CONFIG_UPDATE_ENV_SETTINGS_SCRIPT \
	"mmcargs=setenv bootargs console=${console},${baudrate} ${smp} ${extra} ${video} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} ${smp} ${extra} ${video} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run set_ethernet; " \
		"run netargs; " \
		"tftp ${tftp_dir}/${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if tftp ${fdt_addr} ${tftp_dir}/${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi"

#endif

#endif

