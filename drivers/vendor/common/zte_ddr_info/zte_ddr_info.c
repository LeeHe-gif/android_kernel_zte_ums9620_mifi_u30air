//read emmc ddr infomation.

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/kprobes.h>
#include <asm/ptrace.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <mmc_manf_info.h>

#define ZTE_BSP_LOG_PREFIX "[ZTE_EMMC_INFO][EMMC]"
#define log_err(fmt, ...) pr_err(ZTE_BSP_LOG_PREFIX "[ERR]" fmt, ##__VA_ARGS__)
#define log_warn(fmt, ...) pr_warn(ZTE_BSP_LOG_PREFIX "[WARN]" fmt, ##__VA_ARGS__)
#define log_info(fmt, ...) pr_info(ZTE_BSP_LOG_PREFIX "[INFO]" fmt, ##__VA_ARGS__)
#define log_debug(fmt, ...) pr_debug(ZTE_BSP_LOG_PREFIX "[DEBUG]" fmt, ##__VA_ARGS__)

static struct mmc_host *host;
static struct mmc_card *card;
static char emmcinfo_buf[256]= {0};

typedef struct _ddr_manf_info {
	int id;
	char *listname;
} ddr_manf_info;

static ddr_manf_info ddr_list[] = {
	{0x01, "Samsung"},
	{0x02, "Infineon"},
	{0x03, "Elpida"},
	{0x04, "Etpon"},
	{0x05, "Nanya"},
	{0x06, "Hynix"},
	{0x07, "Mosel"},
	{0x08, "Winbond"},
	{0x09, "Esmt"},
	{0x13, "Cxmt"},
	{0xff, "Micro"}
};

static int ddr_id_proc_show(struct seq_file *m, void *v)
{
	static char *manfname = "UNKNOWN";
	static char ddr_manfname[32];
	struct device_node *cmdline_node;
	const char *cmdline, *name;
	int ret;
	unsigned long memory;
	char *ddr_type = "LPDDR4";
	int ddr_id;
	int i = 0;

	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);
	if (ret) {
		log_info("%s(): Can't not parse bootargs\n", __func__);
		return 0;
	}
	name =  strstr(cmdline, "zte_mr5=");
	if (!name || !sscanf(name, "zte_mr5=%s*", ddr_manfname)) {
		return 0;
	}

	ddr_id = simple_strtol(ddr_manfname, NULL, 16);
	for (i = 0; i < ARRAY_SIZE(ddr_list); i++) {
		if (ddr_list[i].id == ddr_id) {
			manfname = ddr_list[i].listname;
			break;
		}
	}

	memory = (totalram_pages() << (PAGE_SHIFT - 10)) / (1024*1024);/*1G=1048576K*/
	memory += 1;

	seq_printf(m,
		"%.8s-NA-NA-%dGB-%.8s\n",
		manfname, memory, ddr_type);
	return 0;
}

static int ddr_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ddr_id_proc_show, NULL);
}

static int zte_string_get_size(u64 size, const enum string_size_units units,
		    char *buf, int len)
{
	static const char *const units_10[] = {
		"B", "kB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB", NULL
	};
	static const char *const units_2[] = {
		"B", "KiB", "MiB", "GiB", "TiB", "PiB", "EiB", "ZiB", "YiB",
		NULL
	};
	static const char *const *const units_str[] = {
		[STRING_UNITS_10] = units_10,
		[STRING_UNITS_2] = units_2,
	};
	static const unsigned int divisor[] = {
		[STRING_UNITS_10] = 1000,
		[STRING_UNITS_2] = 1024,
	};
	int i, j;
	u64 remainder = 0, sf_cap;
	char tmp[8];
	tmp[0] = '\0';
	i = 0;
	if (size >= divisor[units]) {
		while (size >= divisor[units] && units_str[units][i]) {
			remainder = do_div(size, divisor[units]);
			i++;
		}
		sf_cap = size;
		for (j = 0; sf_cap*10 < 1000; j++)
			sf_cap *= 10;
		if (j) {
			remainder *= 1000;
			do_div(remainder, divisor[units]);
			snprintf(tmp, sizeof(tmp), ".%02lld",
				 (unsigned long long)remainder);
			tmp[2] = '\0';
		}
	}
	snprintf(buf, len, "%lld%sGB", (unsigned long long)size,
		 tmp);
	return 0;
}

static int emmc_id_proc_show(struct seq_file *m, void *v)
{
	int i;
	char *manfname = "UNKNOWN";
	char cap_str[10];
	char fwrev[32] = {0};
	unsigned int  sectors;

	for (i = 0; i < ARRAY_SIZE(man_list); i++) {
		if (man_list[i].id == card->cid.manfid) {
			manfname = man_list[i].name;
			break;
		}
	}
	sectors = card->ext_csd.sectors;
	zte_string_get_size(((u64)sectors) * 512, STRING_UNITS_10, cap_str, sizeof(cap_str));
	sprintf(fwrev, "0x%*phN", MMC_FIRMWARE_LEN, card->ext_csd.fwrev);

	snprintf(emmcinfo_buf, sizeof(emmcinfo_buf), "%s-%s-%s-%02d/%04d-%s-0x%02x/0x%02x", manfname, card->cid.prod_name, fwrev, card->cid.month, card->cid.year, cap_str, card->ext_csd.device_life_time_est_typ_a, card->ext_csd.device_life_time_est_typ_b);
	seq_printf(m, "%s\n", emmcinfo_buf);
	return 0;
}

static int emmc_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, emmc_id_proc_show, NULL);
}

static const struct file_operations ddr_id_proc_fops = {
	.open		= ddr_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations emmc_id_proc_fops = {
	.open		= emmc_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __kprobes handler_pre(struct kprobe *p, struct pt_regs *regs)
{
	struct mmc_host *host_tmp;
	if (host) {
		return 0;
	}
#if defined(CONFIG_ARM64)
		host_tmp = (struct mmc_host *)(regs_get_kernel_argument(regs, 0));
#elif defined(CONFIG_ARM)
		host_tmp = (struct mmc_host *)(regs->ARM_r0);
#endif

	if (!host_tmp) {
		log_err("%s(): host_tmp is nullptr!\n", __func__);
		snprintf(emmcinfo_buf, sizeof(emmcinfo_buf), "%s", "unknow");
		return 0;
	}
	card = host_tmp->card;
	if (!card) {
		log_err("%s(): card is nullptr!\n", __func__);
		snprintf(emmcinfo_buf, sizeof(emmcinfo_buf), "%s", "unknow");
		return 0;
	} else {
		if (card->type == MMC_TYPE_MMC) {
			host = host_tmp;
		} else {
			snprintf(emmcinfo_buf, sizeof(emmcinfo_buf), "%s", "unknow");
		}
	}
	return 0;
}

static struct kprobe kp = {
	.symbol_name	= "mmc_cqe_start_req",
};

static void zte_emmc_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(zte_emmc_dwork, zte_emmc_work_func);
static void zte_emmc_work_func(struct work_struct *work)
{
	if (host) {
		log_info("%s(): host get ok\n", __func__);
		unregister_kprobe(&kp);
	} else
		schedule_delayed_work(&zte_emmc_dwork, msecs_to_jiffies(500));
}

static int __init ddr_info_init_procfs(void)
{
	struct proc_dir_entry *pde;
	struct device_node *cmdline_node;
	const char *cmdline, *device;
	static char flashtype[32];
	int ret;
	
	kp.pre_handler = handler_pre;
	ret = register_kprobe(&kp);
	if (ret < 0) {
		log_err("%s(): register_kprobe failed\n", __func__);
		return ret;
	}
	schedule_delayed_work(&zte_emmc_dwork, msecs_to_jiffies(500));
	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);
	if (ret) {
		log_info("%s(): Can't not parse bootargs\n", __func__);
		return 0;
	}
	device = strstr(cmdline, "sprdboot.flash=");
	if (!device  || !sscanf(device, "sprdboot.flash=%s*", flashtype)) {
		return 0;
	}
	/* Started by AICoder, pid:4bd6af8049xa48814baf09a3a05616154f05b8bb */
	pde = proc_create("driver/ddr_id", 0444, NULL, &ddr_id_proc_fops);
	if (!pde) {
		log_err("%s(): proc_create failed!\n", __func__);
		goto err1;
	}

	if (strcmp(flashtype, "ufs") == 0) {
		log_err("%s():  No need to create emmc node\n", __func__);
	} 
	else {
		pde = proc_create("driver/emmc_id", 0444, NULL, &emmc_id_proc_fops);
		if (!pde) {
			log_err("%s(): proc_create failed!\n", __func__);
			goto err1;
		}
	}
	/* Ended by AICoder, pid:4bd6af8049xa48814baf09a3a05616154f05b8bb */
	return 0;
err1:
	return -ENOMEM;
}

static void __exit ddr_info_init_exit(void)
{
	log_info("%s(): ddr_info_init_exit done\n", __func__);
}

module_init(ddr_info_init_procfs);
module_exit(ddr_info_init_exit);

MODULE_DESCRIPTION("ZTE get ddr info");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("ZTE");