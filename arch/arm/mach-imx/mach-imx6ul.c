/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/can/platform/flexcan.h>
#include <linux/gpio.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/pm_opp.h>
#include <linux/fec.h>
#include <linux/netdevice.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"

static struct flexcan_platform_data flexcan_pdata[2];
static int flexcan_en_gpio;
static int flexcan_en_active_high;
static int flexcan0_en;
static int flexcan1_en;

static void imx6ulea_flexcan_switch(void)
{
	if (flexcan0_en || flexcan1_en) {
		gpio_set_value_cansleep(flexcan_en_gpio,
					!flexcan_en_active_high);
		gpio_set_value_cansleep(flexcan_en_gpio,
					flexcan_en_active_high);
	} else {
		gpio_set_value_cansleep(flexcan_en_gpio,
					!flexcan_en_active_high);
	}
}

static void imx6ulea_flexcan0_switch(int enable)
{
	flexcan0_en = enable;
	imx6ulea_flexcan_switch();
}

static void imx6ulea_flexcan1_switch(int enable)
{
	flexcan1_en = enable;
	imx6ulea_flexcan_switch();
}

static int __init imx6ulea_flexcan_fixup(void)
{
	struct device_node *np;
	enum of_gpio_flags en_flags, stby_flags;

	np = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
	if (!np)
		return -ENODEV;

	flexcan_en_gpio = of_get_named_gpio_flags(np, "trx-en-gpio", 0, &en_flags);

	if (gpio_is_valid(flexcan_en_gpio) && 
		!gpio_request_one(flexcan_en_gpio, GPIOF_DIR_OUT, "flexcan-trx-en")) {
		/* flexcan 0 & 1 are using the same GPIOs for transceiver */
		flexcan_pdata[0].transceiver_switch = imx6ulea_flexcan0_switch;
		flexcan_pdata[1].transceiver_switch = imx6ulea_flexcan1_switch;
		if (!(en_flags & OF_GPIO_ACTIVE_LOW))
			flexcan_en_active_high = 1;
	}

	return 0;
}

static void __init imx6ul_enet_clk_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6ul-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6UL_GPR1_ENET_CLK_DIR,
				   IMX6UL_GPR1_ENET_CLK_OUTPUT);
	else
		pr_err("failed to find fsl,imx6ul-iomux-gpr regmap\n");

}

static int ksz8081_phy_fixup(struct phy_device *dev)
{
	if (dev && dev->interface == PHY_INTERFACE_MODE_MII) {
		phy_write(dev, 0x1f, 0x8110);
		phy_write(dev, 0x16, 0x201);
	} else if (dev && dev->interface == PHY_INTERFACE_MODE_RMII) {
		phy_write(dev, 0x1f, 0x8190);
		phy_write(dev, 0x16, 0x202);
	}

	return 0;
}

#define PHY_ID_KSZ8081	0x00221560
static void __init imx6ul_enet_phy_init(void)
{
	phy_register_fixup_for_uid(PHY_ID_KSZ8081, 0xffffffff,	ksz8081_phy_fixup);
}

static inline void imx6ul_enet_init(void)
{
	imx6ul_enet_clk_init();
	imx6ul_enet_phy_init();
	imx6_enet_mac_init("fsl,imx6ul-fec");
}

static void __init imx6ul_init_machine(void)
{
	struct device *parent;

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table,
					NULL, parent);

	imx6ul_enet_init();
	imx_anatop_init();
	imx6ul_pm_init();
}

static void __init imx6ul_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static const char *imx6ul_dt_compat[] __initdata = {
	"fsl,imx6ul",
	NULL,
};

static void __init imx6ul_init_late(void)
{
	platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);

	if (of_machine_is_compatible("fsl,imx6ulea-com"))
		imx6ulea_flexcan_fixup();

	imx6ul_cpuidle_init();
}

static void __init imx6ul_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
#ifdef CONFIG_CPU_FREQ
	imx_busfreq_map_io();
#endif
}

DT_MACHINE_START(IMX6UL, "Freescale i.MX6 UltraLite (Device Tree)")
	.map_io		= imx6ul_map_io,
	.init_irq	= imx6ul_init_irq,
	.init_machine	= imx6ul_init_machine,
	.init_late	= imx6ul_init_late,
	.dt_compat	= imx6ul_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
