#include <linux/printk.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/string.h>
#include "tegra210_i2s_alt.h"

static int demo_init(void)
{
	printk(KERN_ERR "<demo_dbg> demo_init\n");
	tegra210_i2s_enable(4);
	mdelay(100);
	//tegra210_i2s_disable(4);
	printk(KERN_ERR "<demo_dbg> disable\n");
	return 0;
}

static void demo_exit(void)
{
}

late_initcall(demo_init);
module_exit(demo_exit);
