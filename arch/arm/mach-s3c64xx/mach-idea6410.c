/* linux/arch/arm/mach-s3c64xx/mach-idea6410.c
 *
 * Copyright 2010 Promwad Innovation Company
 *	Yauhen Kharuzhy <yauhen.kharuzhy@promwad.com>
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/dm9000.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <video/platform_lcd.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/regs-fb.h>
#include <mach/map.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-modem.h>
#include <mach/regs-gpio.h>
#include <mach/regs-sys.h>
#include <mach/regs-srom.h>
#include <plat/ata.h>
#include <plat/iic.h>
#include <plat/fb.h>
#include <plat/gpio-cfg.h>
#include <plat/nand.h>

#include <mach/s3c6410.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/keypad.h>

/* DM9000 */
#define IDEA6410_PA_DM9000	(0x18000000)

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg idea6410_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
};

static struct mtd_partition idea6410_nand_part[] = {
	[0] = {
		.name	= "uboot",
		.size	= SZ_1M,
		.offset	= 0,
	},
	[1] = {
		.name	= "kernel",
		.size	= SZ_2M,
		.offset	= MTDPART_OFS_NXTBLK,
	},
	[2] = {
		.name	= "rootfs",
		.size	= MTDPART_SIZ_FULL,
		.offset	= MTDPART_OFS_NXTBLK,
	},
};

static struct s3c2410_nand_set idea6410_nand_sets[] = {
	[0] = {
		.name		= "nand",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(idea6410_nand_part),
		.partitions	= idea6410_nand_part,
	},
};

static struct s3c2410_platform_nand idea6410_nand_info = {
	.tacls		= 26,
	.twrph0		= 25,
	.twrph1		= 25,
	.nr_sets	= ARRAY_SIZE(idea6410_nand_sets),
	.sets		= idea6410_nand_sets,
};


/* framebuffer and LCD setup. */

/* GPF15 = LCD backlight control
 * GPF13 => Panel power
 * GPN5 = LCD nRESET signal
 * PWM_TOUT1 => backlight brightness
 */

static void idea6410_lcd_power_set(struct plat_lcd_data *pd,
				   unsigned int power)
{
	if (power) {
	} else {
	}
}

static struct plat_lcd_data idea6410_lcd_power_data = {
	.set_power	= idea6410_lcd_power_set,
};

static struct platform_device idea6410_lcd_powerdev = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &idea6410_lcd_power_data,
};

static struct s3c_fb_pd_win idea6410_fb_win0 = {
	/* this is to ensure we use win0 */
	.win_mode	= {
		.left_margin	= 8,
		.right_margin	= 13,
		.upper_margin	= 7,
		.lower_margin	= 5,
		.hsync_len	= 3,
		.vsync_len	= 1,
		.xres		= 800,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 16,
	.virtual_y	= 480 * 2,
	.virtual_x	= 800,
};

static struct s3c_fb_platdata idea6410_lcd_pdata __initdata = {
	.setup_gpio	= s3c64xx_fb_gpio_setup_24bpp,
	.win[0]		= &idea6410_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
};

static uint32_t idea6410_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 3, KEY_1), KEY(0, 4, KEY_2), KEY(0, 5, KEY_3),
	KEY(0, 6, KEY_4), KEY(0, 7, KEY_5),
	KEY(1, 3, KEY_A), KEY(1, 4, KEY_B), KEY(1, 5, KEY_C),
	KEY(1, 6, KEY_D), KEY(1, 7, KEY_E)
};

static struct matrix_keymap_data idea6410_keymap_data __initdata = {
	.keymap		= idea6410_keymap,
	.keymap_size	= ARRAY_SIZE(idea6410_keymap),
};

static struct samsung_keypad_platdata idea6410_keypad_data __initdata = {
	.keymap_data	= &idea6410_keymap_data,
	.rows		= 2,
	.cols		= 8,
};

static struct resource idea6410_dm9000_resource[] = {
	[0] = {
		.start = S3C64XX_PA_XM0CSN1,
		.end   = S3C64XX_PA_XM0CSN1 + 3,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = S3C64XX_PA_XM0CSN1 + 4,
		.end   = S3C64XX_PA_XM0CSN1 + 5,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_EINT(7),
		.end   = IRQ_EINT(7),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct dm9000_plat_data idea6410_dm9000_pdata = {
	.flags	  = (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
	/* dev_addr can be set to provide hwaddr. */
};

static struct platform_device idea6410_device_eth = {
	.name	= "dm9000",
	.id	= -1,
	.num_resources	= ARRAY_SIZE(idea6410_dm9000_resource),
	.resource	= idea6410_dm9000_resource,
	.dev	= {
		.platform_data  = &idea6410_dm9000_pdata,
	},
};

static struct gpio_led idea6410_leds[] = {
	{
		.name		= "LED1:blue",
		.gpio		= S3C64XX_GPM(0),
		.active_low	= 1,
		.default_state	= LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name		= "LED2:blue",
		.gpio		= S3C64XX_GPM(1),
		.active_low	= 1,
		.default_state	= LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name		= "LED3:blue",
		.gpio		= S3C64XX_GPM(2),
		.active_low	= 1,
		.default_state	= LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name		= "LED4:blue",
		.gpio		= S3C64XX_GPM(3),
		.active_low	= 1,
		.default_state	= LEDS_GPIO_DEFSTATE_ON,
	},
};

static struct gpio_led_platform_data idea6410_leds_pdata = {
	.num_leds	= ARRAY_SIZE(idea6410_leds),
	.leds		= idea6410_leds,
};

static struct platform_device idea6410_leds_dev = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &idea6410_leds_pdata,
	},
};

static struct platform_device s3c_device_pwm = {
	.name	= "s3c24xx-pwm",
	.id	= -1,
};

static struct platform_device idea6410_device_pwm_speaker = {
	.name	= "pwm-beeper",
	.id	= -1,
	.dev	= {
		.platform_data = (void *)1UL,
	},
};

static struct map_desc idea6410_iodesc[] = {};

static struct platform_device *idea6410_devices[] __initdata = {
	&s3c_device_nand,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_fb,
	&s3c_device_ohci,
	&s3c_device_usb_hsotg,
	&samsung_device_keypad,
	&idea6410_lcd_powerdev,
	&s3c_device_adc,
	&s3c_device_rtc,
	&s3c_device_ts,
	&s3c_device_wdt,
	&s3c_device_pwm,
	&idea6410_device_eth,
	&idea6410_leds_dev,
//	&idea6410_device_pwm_speaker,
};

static struct i2c_board_info i2c_devs0[] __initdata = {
};

static struct i2c_board_info i2c_devs1[] __initdata = {
};

static struct s3c2410_ts_mach_info s3c_ts_platform __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
};

static void __init idea6410_map_io(void)
{
	u32 tmp;

	s3c64xx_init_io(idea6410_iodesc, ARRAY_SIZE(idea6410_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(idea6410_uartcfgs, ARRAY_SIZE(idea6410_uartcfgs));

	/* set the LCD type */

	tmp = __raw_readl(S3C64XX_SPCON);
	tmp &= ~S3C64XX_SPCON_LCD_SEL_MASK;
	tmp |= S3C64XX_SPCON_LCD_SEL_RGB;
	__raw_writel(tmp, S3C64XX_SPCON);

	/* remove the lcd bypass */
	tmp = __raw_readl(S3C64XX_MODEM_MIFPCON);
	tmp &= ~MIFPCON_LCD_BYPASS;
	__raw_writel(tmp, S3C64XX_MODEM_MIFPCON);
}

static void __init idea6410_machine_init(void)
{
	u32 cs1;

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);
	s3c_fb_set_platdata(&idea6410_lcd_pdata);

	samsung_keypad_set_platdata(&idea6410_keypad_data);

	s3c_nand_set_platdata(&idea6410_nand_info);
	s3c24xx_ts_set_platdata(&s3c_ts_platform);

	/* configure nCS1 width to 16 bits */

	cs1 = __raw_readl(S3C64XX_SROM_BW) &
		    ~(S3C64XX_SROM_BW__CS_MASK << S3C64XX_SROM_BW__NCS1__SHIFT);
	cs1 |= ((1 << S3C64XX_SROM_BW__DATAWIDTH__SHIFT) |
		(1 << S3C64XX_SROM_BW__WAITENABLE__SHIFT) |
		(1 << S3C64XX_SROM_BW__BYTEENABLE__SHIFT)) <<
						   S3C64XX_SROM_BW__NCS1__SHIFT;
	__raw_writel(cs1, S3C64XX_SROM_BW);

	/* set timing for nCS1 suitable for ethernet chip */

	__raw_writel((0 << S3C64XX_SROM_BCX__PMC__SHIFT) |
		     (6 << S3C64XX_SROM_BCX__TACP__SHIFT) |
		     (4 << S3C64XX_SROM_BCX__TCAH__SHIFT) |
		     (1 << S3C64XX_SROM_BCX__TCOH__SHIFT) |
		     (0xe << S3C64XX_SROM_BCX__TACC__SHIFT) |
		     (4 << S3C64XX_SROM_BCX__TCOS__SHIFT) |
		     (0 << S3C64XX_SROM_BCX__TACS__SHIFT), S3C64XX_SROM_BC1);

	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	/* Beeper */
//	s3c_gpio_cfgpin(S3C64XX_GPF(15), S3C_GPIO_SFN(2));

	platform_add_devices(idea6410_devices, ARRAY_SIZE(idea6410_devices));
}

MACHINE_START(IDEA6410, "IDEA6410")
	/* Maintainer: Yauhen Kharuzhy <jekhor@gmail.com> */
	.boot_params	= S3C64XX_PA_SDRAM + 0x100,

	.init_irq	= s3c6410_init_irq,
	.map_io		= idea6410_map_io,
	.init_machine	= idea6410_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
