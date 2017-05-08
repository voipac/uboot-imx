/*
 * Copyright (C) 2015-2017 Voipac.
 *
 * Configuration settings for the Fedevel i.MX6 REX board.
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#ifndef MX6_REX_ANDROID_COMMON_H
#define MX6_REX_ANDROID_COMMON_H

#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED

#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW           2

#define CONFIG_G_DNL_VENDOR_NUM               0x18d1
#define CONFIG_G_DNL_PRODUCT_NUM              0x0d02
#define CONFIG_G_DNL_MANUFACTURER             "FSL"

#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_FASTBOOT_FLASH

#define CONFIG_FSL_FASTBOOT
#define CONFIG_ANDROID_RECOVERY

#define CONFIG_FASTBOOT_STORAGE_MMC

#define CONFIG_ANDROID_BOOT_PARTITION_MMC     1
#define CONFIG_ANDROID_SYSTEM_PARTITION_MMC   5
#define CONFIG_ANDROID_RECOVERY_PARTITION_MMC 2
#define CONFIG_ANDROID_CACHE_PARTITION_MMC    6
#define CONFIG_ANDROID_DATA_PARTITION_MMC     4

#define CONFIG_CMD_BOOTA
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_SERIAL_TAG

#define CONFIG_USB_FASTBOOT_BUF_ADDR          CONFIG_SYS_LOAD_ADDR
#define CONFIG_USB_FASTBOOT_BUF_SIZE          0x19000000

#endif /* MX6_REX_ANDROID_COMMON_H */
