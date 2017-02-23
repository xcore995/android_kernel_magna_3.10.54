#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/mtk_rtc.h>
#include <mach/wd_api.h>
/*                                                */
/*                                                                            */
#ifdef CONFIG_LGE_HANDLE_PANIC
#include <mach/board_lge.h>
#include <mach/mt_reg_base.h>
extern int hreset_enable;
extern int on_user_build;
#endif
/*                                                */

extern void wdt_arch_reset(char);



void arch_reset(char mode, const char *cmd)
{
	char reboot = 0;
#ifdef CONFIG_LGE_HANDLE_PANIC
#if 1
    volatile u32 *lge_boot_magic_number = (volatile u32*)(INTER_SRAM + 0xFC00 - 32);
#else
    volatile u32 *lge_boot_magic_number = (volatile u32*)(INTER_SRAM + 0xF800 - 32);
#endif
	volatile u32 *lge_boot_reason       = lge_boot_magic_number + 1; // next 4 byte
#else
	int res = 0;
	struct wd_api *wd_api = NULL;

	res = get_wd_api(&wd_api);
#endif

	pr_notice("arch_reset: cmd = %s\n", cmd ? : "NULL");

	if (cmd && !strcmp(cmd, "charger")) {
		/* do nothing */
	} else if (cmd && !strcmp(cmd, "recovery")) {
		rtc_mark_recovery();
	} else if (cmd && !strcmp(cmd, "bootloader")) {
		rtc_mark_fast();
	}
#ifdef USE_ENTER_DLOAD_SRAM_MAGIC_CODE_
    else if (cmd && !strcmp(cmd, "dload")){
        lge_save_boot_reason(LGE_BOOT_DLOAD_REBOOT, 0, 0);
    }
    else if (cmd && !strcmp(cmd, "oem-90466252")){
        lge_save_boot_reason(LGE_BOOT_LAF_RESTART_REBOOT, 0, 0);
        reboot = 1;
    }
#endif

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	else if (cmd && !strcmp(cmd, "kpoc")) {
		rtc_mark_kpoc();
	}
#endif
#ifdef CONFIG_LGE_PM_CHARGERLOGO
	else if (cmd && !strcmp(cmd, "charge_reset")) {
		reboot = 1;
	}
#endif
#if defined(CONFIG_LGE_HANDLE_PANIC) || defined(CONFIG_LGE_HIDDEN_RESET)
//                                          
    else if (cmd && !strcmp(cmd, "fota")) {
        lge_save_boot_reason(LGE_BOOT_FOTA_REBOOT, 0, 0);
	reboot = 1;
    }
//                                          
#endif
	else {
		reboot = 1;
	}
#ifdef CONFIG_LGE_HANDLE_PANIC
	if (*lge_boot_magic_number == LGE_BOOT_REASON_MAGIC_CODE &&
			*lge_boot_reason == LGE_BOOT_KERNEL_CRASH)
	{
		if (hreset_enable == 0)
		{
			//TODO: lcd display crash log
		} else if ((on_user_build == 1 && hreset_enable == -1) || (hreset_enable == 1))
		{
			lge_save_boot_reason(LGE_BOOT_HIDDEN_RESET_REBOOT, 0, 0);
		}
	}

	printk(KERN_DEBUG "%s : *lge_boot_magic_number = 0x%x, *lge_boot_reason = 0x%x\n", __func__, *lge_boot_magic_number, *lge_boot_reason);
	printk(KERN_DEBUG "%s : lge_boot_magic_number = 0x%p, lge_boot_reason = 0x%p\n", __func__, lge_boot_magic_number, lge_boot_reason);
	wdt_arch_reset(reboot);
#else
	if (res) {
		pr_notice("arch_reset, get wd api error %d\n", res);
	} else {
		wd_api->wd_sw_reset(reboot);
	}
#endif
}
