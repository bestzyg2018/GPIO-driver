/*
 * Xilinx Zynq GPIO device driver
 *
 * Copyright (C) 2009 - 2014 Xilinx, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h> 
#include <linux/jiffies.h> 
#include <linux/irq.h>
#include "fifo.h"
#include "head.h"





#define DRIVER_NAME "zynq-gpio"

/* Maximum banks */
#define ZYNQ_GPIO_MAX_BANK	4
#define ZYNQMP_GPIO_MAX_BANK	6
#define VERSAL_GPIO_MAX_BANK	4
#define VERSAL_UNUSED_BANKS	2

#define ZYNQ_GPIO_BANK0_NGPIO	32
#define ZYNQ_GPIO_BANK1_NGPIO	22
#define ZYNQ_GPIO_BANK2_NGPIO	32
#define ZYNQ_GPIO_BANK3_NGPIO	32

#define ZYNQMP_GPIO_BANK0_NGPIO 26
#define ZYNQMP_GPIO_BANK1_NGPIO 26
#define ZYNQMP_GPIO_BANK2_NGPIO 26
#define ZYNQMP_GPIO_BANK3_NGPIO 32
#define ZYNQMP_GPIO_BANK4_NGPIO 32
#define ZYNQMP_GPIO_BANK5_NGPIO 32

#define	ZYNQ_GPIO_NR_GPIOS	118
#define	ZYNQMP_GPIO_NR_GPIOS	174

#define ZYNQ_GPIO_BANK0_PIN_MIN(str)	0
#define ZYNQ_GPIO_BANK0_PIN_MAX(str)	(ZYNQ_GPIO_BANK0_PIN_MIN(str) + \
					ZYNQ##str##_GPIO_BANK0_NGPIO - 1)
#define ZYNQ_GPIO_BANK1_PIN_MIN(str)	(ZYNQ_GPIO_BANK0_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK1_PIN_MAX(str)	(ZYNQ_GPIO_BANK1_PIN_MIN(str) + \
					ZYNQ##str##_GPIO_BANK1_NGPIO - 1)
#define ZYNQ_GPIO_BANK2_PIN_MIN(str)	(ZYNQ_GPIO_BANK1_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK2_PIN_MAX(str)	(ZYNQ_GPIO_BANK2_PIN_MIN(str) + \
					ZYNQ##str##_GPIO_BANK2_NGPIO - 1)
#define ZYNQ_GPIO_BANK3_PIN_MIN(str)	(ZYNQ_GPIO_BANK2_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK3_PIN_MAX(str)	(ZYNQ_GPIO_BANK3_PIN_MIN(str) + \
					ZYNQ##str##_GPIO_BANK3_NGPIO - 1)
#define ZYNQ_GPIO_BANK4_PIN_MIN(str)	(ZYNQ_GPIO_BANK3_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK4_PIN_MAX(str)	(ZYNQ_GPIO_BANK4_PIN_MIN(str) + \
					ZYNQ##str##_GPIO_BANK4_NGPIO - 1)
#define ZYNQ_GPIO_BANK5_PIN_MIN(str)	(ZYNQ_GPIO_BANK4_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK5_PIN_MAX(str)	(ZYNQ_GPIO_BANK5_PIN_MIN(str) + \
					ZYNQ##str##_GPIO_BANK5_NGPIO - 1)

/* Register offsets for the GPIO device */
/* LSW Mask & Data -WO */
#define ZYNQ_GPIO_DATA_LSW_OFFSET(BANK)	(0x000 + (8 * BANK))
/* MSW Mask & Data -WO */
#define ZYNQ_GPIO_DATA_MSW_OFFSET(BANK)	(0x004 + (8 * BANK))
/* Data Register-RW */
#define ZYNQ_GPIO_DATA_OFFSET(BANK)	(0x040 + (4 * BANK))
#define ZYNQ_GPIO_DATA_RO_OFFSET(BANK)	(0x060 + (4 * BANK))
/* Direction mode reg-RW */
#define ZYNQ_GPIO_DIRM_OFFSET(BANK)	(0x204 + (0x40 * BANK))
/* Output enable reg-RW */
#define ZYNQ_GPIO_OUTEN_OFFSET(BANK)	(0x208 + (0x40 * BANK))
/* Interrupt mask reg-RO */
#define ZYNQ_GPIO_INTMASK_OFFSET(BANK)	(0x20C + (0x40 * BANK))
/* Interrupt enable reg-WO */
#define ZYNQ_GPIO_INTEN_OFFSET(BANK)	(0x210 + (0x40 * BANK))
/* Interrupt disable reg-WO */
#define ZYNQ_GPIO_INTDIS_OFFSET(BANK)	(0x214 + (0x40 * BANK))
/* Interrupt status reg-RO */
#define ZYNQ_GPIO_INTSTS_OFFSET(BANK)	(0x218 + (0x40 * BANK))
/* Interrupt type reg-RW */
#define ZYNQ_GPIO_INTTYPE_OFFSET(BANK)	(0x21C + (0x40 * BANK))
/* Interrupt polarity reg-RW */
#define ZYNQ_GPIO_INTPOL_OFFSET(BANK)	(0x220 + (0x40 * BANK))
/* Interrupt on any, reg-RW */
#define ZYNQ_GPIO_INTANY_OFFSET(BANK)	(0x224 + (0x40 * BANK))

/* Disable all interrupts mask */
#define ZYNQ_GPIO_IXR_DISABLE_ALL	0xFFFFFFFF

/* Mid pin number of a bank */
#define ZYNQ_GPIO_MID_PIN_NUM 16

/* GPIO upper 16 bit mask */
#define ZYNQ_GPIO_UPPER_MASK 0xFFFF0000

/* set to differentiate zynq from zynqmp, 0=zynqmp, 1=zynq */
#define ZYNQ_GPIO_QUIRK_IS_ZYNQ	BIT(1) /*BIT(0)*/
#define GPIO_QUIRK_DATA_RO_BUG	BIT(1)
#define GPIO_QUIRK_VERSAL	BIT(2)


static struct class* gpio_driver_class;
static struct device* gpio_driver_device;

static int zynq_gpio_open(struct inode *inode, struct file *filp)
{
    printk("zynq_gpio open\n");
    return 0;
	
}


static void zynq_gpio_set_value(unsigned int bank, unsigned int pin, int state)
{
	unsigned int reg_offset, bank_num, bank_pin_num;

    	bank_num = bank;
	bank_pin_num = pin;
	//zynq_gpio_get_bank_pin(pin, &bank_num, &bank_pin_num, gpio);
	

	if (bank_pin_num >= ZYNQ_GPIO_MID_PIN_NUM) {
		// only 16 data bits in bit maskable reg
		bank_pin_num -= ZYNQ_GPIO_MID_PIN_NUM;
		reg_offset = ZYNQ_GPIO_DATA_MSW_OFFSET(bank_num);
	} else {
		reg_offset = ZYNQ_GPIO_DATA_LSW_OFFSET(bank_num);
	}

	/* get the 32 bit value to be written to the mask/data register where
	 * the upper 16 bits is the mask and lower 16 bits is the data
	 */
	state = !!state;
	state = ~(1 << (bank_pin_num + ZYNQ_GPIO_MID_PIN_NUM)) &
		((state << bank_pin_num) | ZYNQ_GPIO_UPPER_MASK);
	writel_relaxed(state, fpga_base + reg_offset);
	
}

static int zynq_gpio_dir_out(unsigned int bank, unsigned int pin, unsigned int state)
{
	u32 reg;
	unsigned int bank_num, bank_pin_num;

	//zynq_gpio_get_bank_pin(pin, &bank_num, &bank_pin_num, gpio);

	/* set the GPIO pin as output */
	bank_num = bank;
	bank_pin_num = pin;
	reg = readl_relaxed(fpga_base + ZYNQ_GPIO_DIRM_OFFSET(bank_num));
	reg |= BIT(bank_pin_num);
	
	writel_relaxed(reg, fpga_base + ZYNQ_GPIO_DIRM_OFFSET(bank_num));

	/* configure the output enable reg for the pin */
	reg = readl_relaxed(fpga_base + ZYNQ_GPIO_OUTEN_OFFSET(bank_num));
	reg |= BIT(bank_pin_num);

	writel_relaxed(reg, fpga_base + ZYNQ_GPIO_OUTEN_OFFSET(bank_num));
	//spin_unlock_irqrestore(&gpio->dirlock, flags);

	/* set the state of the pin */
	zynq_gpio_set_value(bank, pin, state);
	return 0;
}


static ssize_t zynq_gpio_write(struct file *filp, const char __user *buff,size_t count, loff_t *f_pos)
{

	char buff_io[10];

	int bank_num, bank_pin_num,val;
        int len;

	
	printk("write gpio\n");
	
	len = copy_from_user(&buff_io,buff,count);
	if(len != 0){
	    printk("copy_from_user failed\n");

	}
	

	bank_num = (int)buff_io[0];
	bank_pin_num = (int)buff_io[1];
	val = (int)buff_io[2];	
	

        printk("bank_num %d,bank_pin_num %d, val,%d\n",bank_num,bank_pin_num,val);


	zynq_gpio_dir_out(bank_num,bank_pin_num,val);
        
   	return 0;
}

static const struct file_operations fpga_fops = 
{
	.owner = THIS_MODULE,
	.open = zynq_gpio_open, 
	.write = zynq_gpio_write,
};

static int __init zynq_gpio_init_cdev(void)
{

    printk("ZYNQ GPIO init \n");

    major = register_chrdev(0, "ZYNQ-GPIO", &fpga_fops); // ×¢²á, ¸æËßÄÚºË
    if (major < 0){
        printk("failed to register device.\n");
        return -1;
    }
    gpio_driver_class = class_create(THIS_MODULE, "ZYNQ-GPIO");
      if (IS_ERR(gpio_driver_class)){
        printk("failed to create pwm moudle class.\n");
        unregister_chrdev(major, "ZYNQ-GPIO");
        return -1;
    }
    gpio_driver_device = device_create(gpio_driver_class, NULL, MKDEV(major, 0), NULL, "ZYNQ-GPIO"); 
    if (IS_ERR(gpio_driver_device)){
        printk("failed to create device .\n");
        unregister_chrdev(major, "ZYNQ-GPIO");
        return -1;
    }
	

	gpio_configure();

	return 0;
}

void gpio_configure(void)
{

	mem_base=0xE000A000;	
	fpga_base = ioremap(mem_base, ZYNQ_GPIO_BANK0_NGPIO+ZYNQ_GPIO_BANK1_NGPIO+ZYNQ_GPIO_BANK2_NGPIO+ZYNQ_GPIO_BANK3_NGPIO);

    	CLK =ioremap(0XF800012C,4);
    	iowrite32(0x01ec044d,CLK);//clk en
  	printk("gpio configure");
}


static int __init zynq_gpio_init(void)
{
	        
	printk("init gpio module.\n");
	if (zynq_gpio_init_cdev())
	goto init_fail_1;
	//if (zynq_fpga_init_class())
	//goto init_fail_2;
	return 0;
	//init_fail_2:
        //device_destroy(gpio_driver_class, MKDEV(major, 0));
        //class_unregister(gpio_driver_class);
        //class_destroy(gpio_driver_class);
        //unregister_chrdev(major, "ZYNQ-GPIO");
	init_fail_1:
	return -1;
}

static void __exit zynq_gpio_exit(void)
{
	printk("exit gpio module.\n");
	
	device_destroy(gpio_driver_class, MKDEV(major, 0));
    	class_unregister(gpio_driver_class);
    	class_destroy(gpio_driver_class);
    	unregister_chrdev(major, "ZYNQ-GPIO");
	release_mem_region(mem_base, ZYNQ_GPIO_BANK0_NGPIO+ZYNQ_GPIO_BANK1_NGPIO+ZYNQ_GPIO_BANK2_NGPIO+ZYNQ_GPIO_BANK3_NGPIO); 

	iounmap(fpga_base);
	
}



module_init(zynq_gpio_init);
module_exit(zynq_gpio_exit);

MODULE_AUTHOR("Xilinx Inc.");
MODULE_DESCRIPTION("Zynq GPIO driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("V1.0");

