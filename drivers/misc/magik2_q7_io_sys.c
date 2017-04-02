/*
 *	DISCRETE IO DRIVER WHICH CREATES SYSFS INTERFACE 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
//#include <mach/gpio.h>
#include "../../arch/arm/mach-imx/hardware.h"
#define DEV_NAME "magik2_q7_io_sys"

#define MAGIK2_Q7_LCD_BKL               IMX_GPIO_NR(1, 9)
#define MAGIK2_Q7_LCD_BKL2              IMX_GPIO_NR(1, 0)


struct magik2_q7{
	struct device dev;
} magik2_q7_obj;

static struct class *magik2_q7_class;


static int lcd_bkl_show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	int flag1 = 0;
	flag1 = gpio_get_value(MAGIK2_Q7_LCD_BKL);
	return sprintf(buf, "%d\n", flag1);
}

static int lcd_bkl_store_status(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value ;
	int rc;
	rc = kstrtoul(buf, 0, &value);
	if (rc <0) 
		return rc;
	gpio_set_value(MAGIK2_Q7_LCD_BKL, value);
	mdelay(1);

	return 1;
}

static int lcd_bkl2_show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	int flag1 = 0;
	flag1 = gpio_get_value(MAGIK2_Q7_LCD_BKL2);
	return sprintf(buf, "%d\n", flag1);
}

static int lcd_bkl2_store_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value ;
	int rc;
	rc = kstrtoul(buf, 0, &value);
	if (rc <0) 
		return rc;
	gpio_set_value(MAGIK2_Q7_LCD_BKL2, value);
	mdelay(1);

	return 1;
}

static DEVICE_ATTR(lcd_bkl, S_IWUSR | S_IRUGO, lcd_bkl_show_status, lcd_bkl_store_status);
static DEVICE_ATTR(lcd_bkl2, S_IWUSR | S_IRUGO, lcd_bkl2_show_status, lcd_bkl2_store_status);

static int __init magik2_q7_sysfs_init(void)
{
	int ret,err;
	magik2_q7_obj.dev.class = magik2_q7_class;
	dev_set_name(&magik2_q7_obj.dev, DEV_NAME);

	err = gpio_request(MAGIK2_Q7_LCD_BKL, "lcd_bkl");
	if (err)
		return err;

	err = gpio_direction_output(MAGIK2_Q7_LCD_BKL, 1);
	if (err)
		return err;

	err = gpio_request(MAGIK2_Q7_LCD_BKL2, "lcd_bkl2");
	if (err)
		return err;

	err = gpio_direction_output(MAGIK2_Q7_LCD_BKL2, 1);
	if (err)
		return err;

	ret = device_register(&magik2_q7_obj.dev);
	if (ret) {
		printk(KERN_ERR "unable to register magik2_q7 in sysfs\n");
		return ret;
	}

	ret = device_create_file(&magik2_q7_obj.dev, &dev_attr_lcd_bkl);
	if (ret) {
		printk(KERN_ERR "unable to create sysfs entry for audio amplifier power\n");
		return ret;
	}

	ret = device_create_file(&magik2_q7_obj.dev, &dev_attr_lcd_bkl2);
	if (ret) {
		printk(KERN_ERR "unable to create sysfs entry for wdog gpio\n");
		return ret;
	}
	return 0;	
}

static void __exit magik2_q7_sysfs_exit(void)
{
	device_remove_file(&magik2_q7_obj.dev, &dev_attr_lcd_bkl);
	device_remove_file(&magik2_q7_obj.dev, &dev_attr_lcd_bkl2);
}


module_init(magik2_q7_sysfs_init);
module_exit(magik2_q7_sysfs_exit);

MODULE_AUTHOR("TESSOLVE");
MODULE_DESCRIPTION("magik2_q7 discrete IO driver");
MODULE_LICENSE("GPL");

