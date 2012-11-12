/*
 * Texas Instruments TCI6614 EVM Board Support
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/irqs.h>
#include <mach/mux.h>
#include <mach/cp_intc.h>
#include <mach/tci6614.h>

static struct at24_platform_data at24_eeprom_data = {
	.byte_len	= 1024 * 1024 / 8,
	.page_size	= 128,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info __initdata i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c1024", 0x50),
		.platform_data = &at24_eeprom_data,
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 100,	/* kHz */
	.bus_delay	= 0,	/* usec */
};

static struct tci6614_device_info evm_device_info __initconst = {
	.i2c_config		= &i2c_pdata,
};

static struct of_device_id tci6614_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus",},
	{ .compatible = "ti,tci6614-bus",},
	{ .compatible = "ti,davinci-aemif", },
	{}
};

static __init void tci6614_evm_board_init(void)
{
	of_platform_populate(NULL, tci6614_dt_match_table, NULL, NULL);

	tci6614_devices_init(&evm_device_info);

	i2c_register_board_info(1, i2c_devices,
			ARRAY_SIZE(i2c_devices));
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init tci6614_evm_console_init(void)
{
	return add_preferred_console("ttyS", 0, "115200");
}
console_initcall(tci6614_evm_console_init);
#endif

static void __init tci6614_evm_map_io(void)
{
	tci6614_init();
}

static const char* tci6614_dt_board_compat[] = {
	"ti,tci6614-evm",
	NULL
};

DT_MACHINE_START(TCI6614_EVM, "TCI6614 EVM")
	.map_io		= tci6614_evm_map_io,
	.init_irq	= tci6614_of_init_irq,
	.timer		= &davinci_timer,
	.init_machine	= tci6614_evm_board_init,
	.dt_compat	= tci6614_dt_board_compat,
	.restart	= tci6614_restart,
MACHINE_END
