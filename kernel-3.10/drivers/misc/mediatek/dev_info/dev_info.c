#include <linux/init.h>        /* For init/exit macros */
#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/byteorder/generic.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/xlog.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/namei.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <asm/uaccess.h>
#include <mach/mt_typedefs.h>
#include <mach/hardware.h>
#include <mach/mt_boot.h>

extern int get_camera_SensorID(int *main_sensorID,int *sub_sensorID);

static struct lcm_info_t {
char vendor_name[16];
char driver_name[32];
char ic_type[16];
};

static struct camera_info_t {
char vendor_name[16];
char sensor_type[16];
short sensor_id;
};

static struct tp_info_t {
char vendor_name[16];
char ic_type[16];
char fw_id[32];
};

static union dev_info_t {
struct lcm_info_t lcm_info;
struct camera_info_t camera_info;
struct tp_info_t tp_info;
};

struct lcm_info_t lcm_info[]={
{"TXD","fl10802_dsi_vdo_6572","fl10802"},
{"COE","ili9806e_dsi_vdo_fwvga","ili9806e"}
};

struct camera_info_t main_camera_info[]={
{"MICROKORE","T8EV5",0x1011}
};

struct camera_info_t sub_camera_info[]={
{"MICROKORE","SP2519",0x2519}
};

extern char *saved_command_line;
bool get_lcm_name(char *lcm_name)
{
    bool ret = FALSE;
    char *p, *q;
    if(saved_command_line==NULL)
	return FALSE;
    p = strstr(saved_command_line, "lcm=");
    if(p == NULL)
    {
        return FALSE;
    }
    
    p += 4;
    if((p - saved_command_line) > strlen(saved_command_line+1))
    {
       return FALSE;
    }
       
    p += 2;
    q = p;
    while(*q != ' ' && *q != '\0')
        q++;
    
    if(q != p)
    {
      strncpy(lcm_name, (const char*)p, (int)(q-p));
      printk(" LCM is %s connected\n",lcm_name);
      return TRUE;
    }

    return ret;
}

// ============================================================ //
static ssize_t show_dev_lcm_info(struct device *dev,struct device_attribute *attr, char *buf)
{    
    char lcm_name_str[64];
    char *p,i;
    
    
    if(get_lcm_name(lcm_name_str))
    {
    	printk( "[DEV_INFO] lcm name : %s\n", lcm_name_str);
	for(i=0;i<sizeof(lcm_info)/sizeof(&lcm_info[0]);i++)
	{
	  if(NULL!=strstr(lcm_name_str,lcm_info[i].driver_name))
		 return snprintf(buf, PAGE_SIZE,"vendor:%s ic:%s\n", lcm_info[i].vendor_name,lcm_info[i].ic_type);
	  else
		printk( "[DEV_INFO] lcm driver name : %s\n", lcm_info[i].driver_name);
	}
    }
    return 0;
}

static DEVICE_ATTR(dev_lcm_info, 0444, show_dev_lcm_info, NULL);

// ============================================================ //
static ssize_t show_dev_camera_info(struct device *dev,struct device_attribute *attr, char *buf)
{   
    char i=0,j=0;
    int main_sensor_id=0,sub_sensor_id=0;
 
    get_camera_SensorID(&main_sensor_id,&sub_sensor_id);
    for(i=0;i<sizeof(main_camera_info)/sizeof(&main_camera_info[0]);i++)
    {
	if(main_sensor_id == main_camera_info[i].sensor_id)
		break;
    }
     for(j=0;j<sizeof(sub_camera_info)/sizeof(&sub_camera_info[0]);j++)
    {
	if(sub_sensor_id == sub_camera_info[j].sensor_id)
		break;
    }	
    return snprintf(buf, PAGE_SIZE,"near:%s-%s front:%s-%s\n", main_camera_info[i].vendor_name,main_camera_info[i].sensor_type,sub_camera_info[j].vendor_name,sub_camera_info[j].sensor_type);
}

static DEVICE_ATTR(dev_camera_info, 0444, show_dev_camera_info, NULL);

//-------------------------------------------------------------------------------------------
static ssize_t show_dev_touchpannel_info(struct device *dev,struct device_attribute *attr, char *buf)
{   
    char touchpannel_info_str[64];  
    printk( "[DEV_INFO] touchpannel info : %s\n", touchpannel_info_str);
    return snprintf(buf, PAGE_SIZE,"%s\n", touchpannel_info_str);
}

static DEVICE_ATTR(dev_touchpannel_info, 0444, show_dev_touchpannel_info, NULL);
// ============================================================ //
static int device_info_probe(struct platform_device *dev)    
{

    int ret_device_file = 0;

    printk( "[device_info_probe] probe\n");
    //Create File For Device UI DEBUG
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_dev_lcm_info);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_dev_camera_info);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_dev_touchpannel_info);
    return 0;
}

static int device_info_remove(struct platform_device *dev)    
{
    printk("[device_info_remove]\n");
    return 0;
}

static int device_info_suspend(struct platform_device *dev, pm_message_t state)    
{

	printk("[device_info_suspend]\n");
	return 0;
}

static int device_info_resume(struct platform_device *dev)
{

	printk("[device_info_resume]\n");
	return 0;
}

//-----------------------------------------------------


struct platform_device device_info__device = {
        .name                = "device_info",
        .id                  = -1,
};


static struct platform_driver device_info_driver = {
    .probe        = device_info_probe,
    .remove       = device_info_remove,
    .suspend      = device_info_suspend,
    .resume       = device_info_resume,
    .driver       = {
        .name = "device_info",
      
    },
};

static int __init device_info_init(void)
{
    int ret;
   
    ret = platform_device_register(&device_info__device);
    if (ret) {
        printk("[device_info_driver] Unable to device register(%d)\n", ret);
        return ret;
    }
      
    ret = platform_driver_register(&device_info_driver);
    if (ret) {
        printk("[device_info_driver] Unable to register driver (%d)\n", ret);
        return ret;
    }
   
    printk("[device_info_driver] Initialization : DONE \n");

    return 0;

}

late_initcall(device_info_init);
MODULE_AUTHOR("Andy gao");
MODULE_DESCRIPTION("Get device information");
MODULE_LICENSE("GPL");

