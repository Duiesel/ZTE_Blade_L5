/******************************************************************************
 * mt6575_hall.c - MT6575 Android Linux Vibrator Device Driver
 * 
 * Copyright 2009-2010 MediaTek Co.,Ltd.
 * 
 * DESCRIPTION:
 *     This file provid the other drivers hall relative functions
 *
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>

//#include "timed_output.h"

//#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <linux/jiffies.h>
#include <linux/timer.h>

#include <mach/mt_typedefs.h>
#include <linux/input.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>

#include <linux/leds.h>
//#include <leds_sw.h>
#include <cust_leds.h>
#include <cust_leds_def.h>

#define HALL_DEVICE	        			"mtk_hall"


/******************************************************************************
Error Code No.
******************************************************************************/
#define RSUCCESS        0

/******************************************************************************
Debug Message Settings
******************************************************************************/

/* Debug message event */
#define DBG_EVT_NONE		0x00000000	/* No event */
#define DBG_EVT_INT			0x00000001	/* Interrupt related event */
#define DBG_EVT_TASKLET		0x00000002	/* Tasklet related event */

#define DBG_EVT_ALL			0xffffffff
 
#define DBG_EVT_MASK      	(DBG_EVT_TASKLET)
#if 0
#define GPIO_HALL_1_PIN         (GPIO16 | 0x80000000)
#define GPIO_HALL_1_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_HALL_1_PIN_M_CLK   GPIO_MODE_04
#define GPIO_HALL_1_PIN_M_EINT   GPIO_HALL_1_PIN_M_GPIO
#define GPIO_HALL_1_PIN_CLK     CLK_OUT2
#define GPIO_HALL_1_PIN_FREQ    GPIO_CLKSRC_NONE

#define CUST_EINT_HALL_1_NUM              16
#define CUST_EINT_HALL_1_DEBOUNCE_CN      256
#define CUST_EINT_HALL_1_TYPE							CUST_EINTF_TRIGGER_LOW
#define CUST_EINT_HALL_1_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_ENABLE
#endif

#if 1
#define MSG(evt, fmt, args...) \
do {	\
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		printk(fmt, ##args); \
	} \
} while(0)

#define MSG_FUNC_ENTRY(f)	MSG(FUC, "<FUN_ENT>: %s\n", __FUNCTION__)
#else
#define MSG(evt, fmt, args...) do{}while(0)
#define MSG_FUNC_ENTRY(f)	   do{}while(0)
#endif


/******************************************************************************
Global Definations
******************************************************************************/
static struct workqueue_struct *hall_queue;
static struct work_struct hall_work;
//static struct hrtimer hall_timer;
static spinlock_t hall_lock;
static int hall_state= 0;  //0: hall open 1: hall close
static int shutdown_flag;
static struct input_dev *kpd_hall_dev;

extern int mt65xx_leds_brightness_set(enum mt65xx_led_type type, enum led_brightness level);
static int hall_setup_eint(void);

static void update_hall(struct work_struct *work)
{
	if(hall_state==0)  //hall open
	{
		printk("hall open.\n");
		//mt65xx_leds_brightness_set(MT65XX_LED_TYPE_RED,255);
		input_report_key(kpd_hall_dev, KEY_OPEN, 1);
		input_report_key(kpd_hall_dev, KEY_OPEN, 0);
		input_sync(kpd_hall_dev);
	}
	else   //hall close
	{	
		printk("hall close.\n");
		//mt65xx_leds_brightness_set(MT65XX_LED_TYPE_RED,0);
		input_report_key(kpd_hall_dev, KEY_CLOSE, 1);
		input_report_key(kpd_hall_dev, KEY_CLOSE, 0);
		input_sync(kpd_hall_dev);
	}

	mt_eint_unmask(CUST_EINT_HALL_1_NUM);
}

static int hall_probe(struct platform_device *pdev)
{

	printk("hall probe enter.\n");
	hall_queue = create_singlethread_workqueue(HALL_DEVICE);
	if(!hall_queue) {
		printk("[hall]Unable to create workqueue\n");
		return -ENODATA;
	}

	INIT_WORK(&hall_work, update_hall);

	kpd_hall_dev = input_allocate_device();
	if (!kpd_hall_dev) 
	{
		printk("[hall]kpd_hall_dev : fail!\n");
		return -ENOMEM;
	}

	//define multi-key keycode
	__set_bit(EV_KEY, kpd_hall_dev->evbit);
	__set_bit(KEY_OPEN, kpd_hall_dev->keybit);
	__set_bit(KEY_CLOSE, kpd_hall_dev->keybit);
    	
	
	kpd_hall_dev->id.bustype = BUS_HOST;
	kpd_hall_dev->name = "HALL";
	if(input_register_device(kpd_hall_dev))
	{
		printk("[hall]kpd_hall_dev register : fail!\n");
	}else
	{
		printk("[hall]kpd_hall_dev register : success!!\n");
	} 
	//spin_lock_init(&hall_lock);
	//shutdown_flag = 0;
	hall_state = 0;

	hall_setup_eint();
	return 0;
}

static int hall_remove(struct platform_device *pdev)
{
	if(hall_queue)
		destroy_workqueue(hall_queue);
	if(kpd_hall_dev)
		input_unregister_device(kpd_hall_dev);
	return 0;
}

/******************************************************************************
Device driver structure
*****************************************************************************/
static struct platform_driver hall_driver = 
{
    .probe	= hall_probe,
    .remove	= hall_remove,
    //.shutdown   = hall_shutdown,
    .driver     = {
    .name = HALL_DEVICE,
    .owner = THIS_MODULE,
    },
};

static struct platform_device hall_device =
{
    .name = "mtk_hall",
    .id   = -1,
};

static ssize_t store_set_hall_int_por(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	if(buf != NULL && size != 0)
	{
		printk("[hall]buf is %s and size is %d \n",buf,(int)size);
		if(buf[0]== '0')
		{
			hall_state=0;
		}else
		{
			hall_state=1;
		}
	}
	return size;
}

static ssize_t show_get_hall_state(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "HALL state = %u\n", hall_state);
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(get_hall_state,      S_IWUSR | S_IRUGO, show_get_hall_state,         NULL);

static DRIVER_ATTR(set_hall_int_por,      S_IWUSR | S_IRUGO, NULL,         store_set_hall_int_por);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *hall_attr_list[] = {
	&driver_attr_get_hall_state,        
	&driver_attr_set_hall_int_por	
};

static int hall_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(hall_attr_list)/sizeof(hall_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, hall_attr_list[idx])))
		{            
			printk("driver_create_file (%s) = %d\n", hall_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}


static void hall_eint_func(void)
{
	int ret=0;
	printk("[hall]hall_eint_func\n");  	
	if(hall_state ==  1 ) 
	{
	/*
	To trigger EINT when the headset was plugged in
	We set the polarity back as we initialed.
	*/
		if (CUST_EINT_HALL_1_POLARITY == CUST_EINT_POLARITY_HIGH){
                                        mt65xx_eint_set_polarity(CUST_EINT_HALL_1_NUM, (1));
		}else{
                                        mt65xx_eint_set_polarity(CUST_EINT_HALL_1_NUM, (0));
		}

#ifdef HALL_SHORT_PLUGOUT_DEBOUNCE
                mt65xx_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, CUST_EINT_ACCDET_DEBOUNCE_CN);
#endif

		/* update the eint status */
		hall_state = 0;

	} 
	else 
	{
	/* 
	To trigger EINT when the headset was plugged out 
	We set the opposite polarity to what we initialed. 
	*/
		if (CUST_EINT_HALL_1_POLARITY == CUST_EINT_POLARITY_HIGH){
                                 mt65xx_eint_set_polarity(CUST_EINT_HALL_1_NUM, !(1));
		}else{
                                mt65xx_eint_set_polarity(CUST_EINT_HALL_1_NUM, !(0));
		}
	/* update the eint status */
#ifdef HALL_SHORT_PLUGOUT_DEBOUNCE
        	//mt_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, HALL_SHORT_PLUGOUT_DEBOUNCE_CN);
                mt65xx_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, CUST_EINT_ACCDET_DEBOUNCE_CN);
#endif        
		hall_state = 1;
	
	}

	ret = queue_work(hall_queue, &hall_work);	
      if(!ret)
      {
  	    //printk("[hall]hall_eint_func: return:%d!\n", ret);  		
      }
}

static int hall_setup_eint(void)
{
	
	/*configure to GPIO function, external interrupt*/
    	printk("[hall]hall_setup_eint\n");
	
	mt_set_gpio_mode(GPIO_HALL_1_PIN, GPIO_HALL_1_PIN_M_EINT);
    	mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
    	mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_ENABLE); //To disable GPIO PULL.
        mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	//mt_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_1_DEBOUNCE_CN);
        mt65xx_eint_set_sens(CUST_EINT_HALL_1_NUM,CUST_EINT_HALL_1_SENSITIVE);
        mt65xx_eint_registration(CUST_EINT_HALL_1_NUM, 0,CUST_EINT_HALL_1_POLARITY, hall_eint_func, 0);
	printk("[hall] hall_eint_num=%d, hall_eint_debounce_en=%d, hall_eint_polarity=%d\n", CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_1_DEBOUNCE_EN, CUST_EINT_HALL_1_POLARITY);
	
	mt_eint_unmask(CUST_EINT_HALL_1_NUM);  
	return 0;
}

/******************************************************************************
 * hall_mod_init
 * 
 * DESCRIPTION:
 *   Register the hall device driver ! 
 * 
 * PARAMETERS: 
 *   None
 * 
 * RETURNS: 
 *   None
 * 
 * NOTES: 
 *   RSUCCESS : Success
 * 
 ******************************************************************************/

static int __init hall_mod_init(void)
{	
   s32 ret;

   printk("Techain hall driver register \n");
	
   ret = platform_device_register(&hall_device);
   if (ret != 0){
	printk("[hall]Unable to register hall device (%d)\n", ret);
       	return ret;
   }
    ret = platform_driver_register(&hall_driver);

    if(ret) 
    {
	printk("[hall]Unable to register hall driver (%d)\n", ret);
    }	
    
     if((ret = hall_create_attr(&hall_driver.driver))!=0)
     {
	printk("create attribute err = %d\n", ret);
	
     }
     
     printk("[hall]hall_mod_init Done \n");
 
    return RSUCCESS;
}

/******************************************************************************
 * hall_mod_exit
 * 
 * DESCRIPTION: 
 *   Free the device driver ! 
 * 
 * PARAMETERS: 
 *   None
 * 
 * RETURNS: 
 *   None
 * 
 * NOTES: 
 *   None
 * 
 ******************************************************************************/
 
static void __init hall_mod_exit(void)
{
	platform_driver_unregister(&hall_driver);
	printk("Techainhall driver unregister\n");
	
}

module_init(hall_mod_init);
module_exit(hall_mod_exit);
MODULE_AUTHOR("Techain Inc.");
MODULE_DESCRIPTION("Techain hall driver");
MODULE_LICENSE("GPL");
