/*
 * Copyright (C) 2016, STMicroelectronics - All Rights Reserved
 * Author(s): Vikas Manocha, <vikas.manocha@st.com> for STMicroelectronics.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <ram.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/armv7m.h>
#include <asm/arch/stm32.h>
#include <asm/arch/gpio.h>
#include <asm/arch/stm32_periph.h>
#include <asm/arch/stm32_defs.h>
#include <asm/arch/syscfg.h>
#include <asm/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

void GPIO_Set(struct stm32_gpio_regs* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD)
{
	u32 pinpos=0,pos=0,curpin=0;
	for(pinpos=0;pinpos<16;pinpos++)
	{
		pos=1<<pinpos;	//一个个位检查
		curpin=BITx&pos;//检查引脚是否要设置
		if(curpin==pos)	//需要设置
		{
			GPIOx->moder&=~(3<<(pinpos*2));	//先清除原来的设置
			GPIOx->moder|=MODE<<(pinpos*2);	//设置新的模式
			if((MODE==0X01)||(MODE==0X02))	//如果是输出模式/复用功能模式
			{
				GPIOx->ospeedr&=~(3<<(pinpos*2));	//清除原来的设置
				GPIOx->ospeedr|=(OSPEED<<(pinpos*2));//设置新的速度值
				GPIOx->otyper&=~(1<<pinpos) ;		//清除原来的设置
				GPIOx->otyper|=OTYPE<<pinpos;		//设置新的输出模式
			}
			GPIOx->pupdr&=~(3<<(pinpos*2));	//先清除原来的设置
			GPIOx->pupdr|=PUPD<<(pinpos*2);	//设置新的上下拉
		}
	}
}
void GPIO_Pin_Set(struct stm32_gpio_regs* GPIOx,u16 pinx,u8 status)
{
	if(status&0X01)
		GPIOx->bsrr=pinx;	//设置GPIOx的pinx为1
	else
		GPIOx->bsrr=pinx<<16;			//设置GPIOx的pinx为0
}
void coloured_LED_init(void)
{
	STM32_RCC->ahb1enr |= 1 << 1;
	GPIO_Set((struct stm32_gpio_regs*)STM32_GPIOB_BASE, 0x3, 1, 0, 3, 1);
}

void red_led_on(void)
{
	GPIO_Pin_Set((struct stm32_gpio_regs*)STM32_GPIOB_BASE, 0x2, 1);
}

void red_led_off(void)
{
	GPIO_Pin_Set((struct stm32_gpio_regs*)STM32_GPIOB_BASE, 0x2, 1);
}

int get_memory_base_size(fdt_addr_t *mr_base, fdt_addr_t *mr_size)
{
	int mr_node;

	mr_node = fdt_path_offset(gd->fdt_blob, "/memory");
	if (mr_node < 0)
		return mr_node;
	*mr_base = fdtdec_get_addr_size_auto_noparent(gd->fdt_blob, mr_node,
						      "reg", 0, mr_size, false);
	debug("mr_base = %lx, mr_size= %lx\n", *mr_base, *mr_size);

	return 0;
}
int dram_init(void)
{
	int rv;
	fdt_addr_t mr_base, mr_size;

#ifndef CONFIG_SUPPORT_SPL
	struct udevice *dev;
	rv = uclass_get_device(UCLASS_RAM, 0, &dev);
	if (rv) {
		debug("DRAM init failed: %d\n", rv);
		return rv;
	}

#endif
	rv = get_memory_base_size(&mr_base, &mr_size);
	if (rv)
		return rv;
	gd->ram_size = mr_size;
	gd->ram_top = mr_base;

	return rv;
}

int dram_init_banksize(void)
{
	fdt_addr_t mr_base, mr_size;
	get_memory_base_size(&mr_base, &mr_size);
	/*
	 * Fill in global info with description of SRAM configuration
	 */
	gd->bd->bi_dram[0].start = mr_base;
	gd->bd->bi_dram[0].size  = mr_size;

	return 0;
}

#ifdef CONFIG_ETH_DESIGNWARE
static int stmmac_setup(void)
{
	clock_setup(SYSCFG_CLOCK_CFG);
	/* Set >RMII mode */
	STM32_SYSCFG->pmc |= SYSCFG_PMC_MII_RMII_SEL;
	clock_setup(STMMAC_CLOCK_CFG);

	return 0;
}

int board_early_init_f(void)
{
	stmmac_setup();

	return 0;
}
#endif

#ifdef CONFIG_SPL_BUILD
#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	debug("SPL: booting kernel\n");
	/* break into full u-boot on 'c' */
	return serial_tstc() && serial_getc() == 'c';
}
#endif

int spl_dram_init(void)
{
	struct udevice *dev;
	int rv;
	rv = uclass_get_device(UCLASS_RAM, 0, &dev);
	if (rv)
		debug("DRAM init failed: %d\n", rv);
	return rv;
}
void spl_board_init(void)
{
	spl_dram_init();
	preloader_console_init();
	arch_cpu_init(); /* to configure mpu for sdram rw permissions */
}
u32 spl_boot_device(void)
{
	return BOOT_DEVICE_XIP;
}

#endif
u32 get_board_rev(void)
{
	return 0;
}

int board_late_init(void)
{
	struct gpio_desc gpio = {};
	int node;

	node = fdt_node_offset_by_compatible(gd->fdt_blob, 0, "st,led1");
	if (node < 0)
		return -1;

	gpio_request_by_name_nodev(offset_to_ofnode(node), "led-gpio", 0, &gpio,
				   GPIOD_IS_OUT);

	if (dm_gpio_is_valid(&gpio)) {
		dm_gpio_set_value(&gpio, 0);
		mdelay(10);
		dm_gpio_set_value(&gpio, 1);
	}

	/* read button 1*/
	node = fdt_node_offset_by_compatible(gd->fdt_blob, 0, "st,button1");
	if (node < 0)
		return -1;

	gpio_request_by_name_nodev(offset_to_ofnode(node), "button-gpio", 0,
				   &gpio, GPIOD_IS_IN);

	if (dm_gpio_is_valid(&gpio)) {
		if (dm_gpio_get_value(&gpio))
			puts("usr button is at HIGH LEVEL\n");
		else
			puts("usr button is at LOW LEVEL\n");
	}

	return 0;
}

int board_init(void)
{
	gd->bd->bi_boot_params = gd->bd->bi_dram[0].start + 0x100;
	return 0;
}
