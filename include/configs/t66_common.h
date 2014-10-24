/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QSABRE_COMMON_CONFIG_H
#define __MX6QSABRE_COMMON_CONFIG_H

#define CONFIG_MX6

#include "mx6_common.h"
#include <linux/sizes.h>

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET

#ifndef CONFIG_CMD_PCI
#define CONFIG_FEC_MXC
#define CONFIG_ETHPRIME			"FEC"
#else
#define CONFIG_RTL8169
#define CONFIG_ETHPRIME			"RTL8169#0"
#endif
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		3
#define CONFIG_IPADDR			192.168.1.103
#define CONFIG_SERVERIP			192.168.1.101
#define CONFIG_NETMASK			255.255.255.0

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		(0 | (IMX_GPIO_NR(4, 9) << 8))
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_BOOTDELAY               0

#define CONFIG_LOADADDR                 0x10800000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#define PROFILE_NAME "profile"
#define ROOTFS_NAME "rootfs"
#define CACHE_NAME "cache"
#define BACKUP_NAME "backup"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"initialized=false\0" \
		"initrd_boot_name=nanduinitrd\0" \
		"system-product-name=und\0" \
		"loadaddr=0x10800000\0" \
		"loadaddr_uinitrd=0x10C00000\0" \
		"ac-type=poweroff\0" \
		"autoload=no\0" \
		"netdev=eth0\0" \
		"ethprime=FEC0\0" \
		"uboot=u-boot.bin\0" \
		"kernel=uImage\0" \
		"nfs_server=192.168.2.12\0" \
		"nfs_path=/work/nfs\0" \
		"console=ttymxc0,115200\0" \
		"fwupdate=0\0" \
		"verify=no\0" \
		"mpuname=Freescale i.MX 6Quad\0" \
		"mpurate=400min\0" \
		"Memory=1024 MB\0" \
		"wol=on\0" \
		"otg=on\0" \
		"dbg=quiet\0" \
		"emmcAttr=install_:::;rootfs_primary::81920:3227647;profile_primary:ext4:3227648:3293183;cache_primary:ext4:3293184:3358719;extend_extended::3358720:;backup_logical:ext4:3358721:3424256;UserData_logical:ext4:3424258:\0" \
		"rootfs_label_name="ROOTFS_NAME"\0" \
		"profile_label_name="PROFILE_NAME"\0" \
		"cache_label_name="CACHE_NAME"\0" \
		"backup_label_name="BACKUP_NAME"\0" \
		"rootargs=root=/dev/mmcblk0p1 rootwait rootfstype=squashfs rw\0" \
		"setatb=setenv atbargs userbutton=$userbutton fwupdate=$fwupdate\0" \
		"vpu_clk=266\0" \
		"fbmem=64M\0" \
		"disp0_res=1024x768M@60\0" \
		"disp0_if=RGB24\0" \
		"disp0_bpp=32\0" \
		"disp0_src=vga\0" \
		"setdispargs=setenv dispargs fbmem=${fbmem} video=mxcfb0:dev=${disp0_src},${disp0_res},if=${disp0_if},bpp=${disp0_bpp}\0" \
		"setmx6args=setenv mx6args mpurate=${mpurate} vpu_clk=${vpu_clk}\0" \
		"kernargs=setenv bootargs console=${console} ${rootargs} ${atbargs} ${dispargs} ${mx6args} ${optargs} ${dbg} ${stage2} wol=${wol} otg=${otg} cpld_boot_detect=${cpld_boot_detect} ldo_active=${ldo_active} enable_coredump\0" \
		"rootargs_mmc=setenv rootargs root=/dev/mmcblk0p1 rootwait rootfstype=squashfs rw\0" \
		"rootargs_nfs=setenv rootargs root=/dev/nfs nfsroot=${nfs_server}:${nfs_path},v3,tcp ip=dhcp\0" \
		"mmcargs=run rootargs_mmc; run setatb; run kernargs\0" \
		"nfsargs=run rootargs_nfs; run kernargs\0" \
		"mmcboot=checkatb; run setmx6args; run setdispargs; run mmcargs; run setnandinitrdname; run mmc_load_kernel; if test \"x${initrd_boot_name}\" = \"xnanduinitrd\"; then run mmc_load_initrd; bootm ${loadaddr} ${rd_loadaddr}; else bootm; fi;\0" \
		"nfsboot=run setmx6args; run setdispargs; run nfsargs; run mmc_load_kernel; bootm\0" \
		"uboot_name=u-boot.emmc\0" \
		"kernel_name=uImage\0" \
		"initrd_name=t66_uInitrd\0" \
		"kernel_part_start=0x5000\0" \
		"initrd_part_start=0xA000\0" \
		"save_mbr=mmc dev 2 0; mmc read ${loadaddr} 0x0 0x2; mmc dev 2 2; mmc open 2 2; mmc write ${loadaddr} 0x0 0x2; mmc close 2 2; mmc bootpart 2 1\0" \
		"restore_mbr=mmc dev 2 2; mmc open 2 2; mmc read ${loadaddr} 0x0 0x2; mmc close 2 2; mmc dev 2 0; mmc write ${loadaddr} 0x0 0x2; mmc bootpart 2 1;\0" \
		"mmc_load_kernel=mmc dev 2 0; mmc loadimg ${loadaddr} ${kernel_part_start}\0" \
		"mmc_load_initrd=mmc dev 2 0; mmc loadimg ${rd_loadaddr} ${initrd_part_start}\0" \
		"setnandinitrdname=if test ${userbutton} != 0 || test ${fwupdate} != 0 " \
			"|| test \"x${ethlo}\" = \"x0xffffffff\"; then setenv initrd_boot_name nanduinitrd; " \
			"else setenv initrd_boot_name nanduminiinitrd; fi;\0" \
		"nfs_getuboot=dhcp && nfs ${loadaddr} ${nfs_server}:${nfs_path}/${uboot_name} && setexpr uboot_size ${filesize} / 200 && setexpr uboot_size ${uboot_size} + 1\0" \
		"nfs_getkernel=dhcp && nfs ${loadaddr} ${nfs_server}:${nfs_path}/${kernel_name} && setexpr kernel_size ${filesize} / 200 && setexpr kernel_size ${kernel_size} + 1\0" \
		"nfs_getinitrd=dhcp && nfs ${rd_loadaddr} ${nfs_server}:${nfs_path}/${initrd_name} && setexpr initrd_size ${filesize} / 200 && setexpr initrd_size ${initrd_size} + 1\0" \
		"update_uboot=run nfs_getuboot && mmc dev 2 1 && mmc open 2 1 && mmc write ${loadaddr} 0x0 ${uboot_size} ; mmc close 2 1\0" \
		"update_kernel=run nfs_getkernel && mmc dev 2 0 && mmc write ${loadaddr} ${kernel_part_start} ${kernel_size}\0" \
		"update_initrd=run nfs_getinitrd && mmc dev 2 0 && mmc write ${rd_loadaddr} ${initrd_part_start} ${initrd_size}\0" \
		"disp_detect=on\0" \
		"cpld_boot_detect=true\0" \
		"ldo_active=on\0" \

#define CONFIG_BOOTCOMMAND \
	"if test ${initialized} = true ; then " \
		"setenv stage2; run mmcboot; " \
	"else " \
		"saveenv; setenv stage2 stage2 init=/init;gpio setdisp0; run setdispargs; run kernargs; printevn bootargs; bootm ${loadaddr} ${loadaddr_uinitrd};" \
	"fi"

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_MISC_INIT_R
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT              "t66#"
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2    "> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              1024	/* Console I/O Buffer Size */

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             32
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SECT_SIZE   (512 * 1024)
#define CONFIG_ENV_SIZE         CONFIG_ENV_SECT_SIZE

#define CONFIG_ENV_IS_IN_MMC

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif                         /* __MX6QSABRE_COMMON_CONFIG_H */
