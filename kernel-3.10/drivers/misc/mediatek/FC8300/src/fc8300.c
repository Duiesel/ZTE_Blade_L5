/*****************************************************************************
	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8300.c

	Description : Driver source file

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/io.h>


#include "fc8300.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8300_regs.h"
#include "fc8300_isr.h"
#include "fci_hal.h"

#ifdef FEATURE_MTK
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <cust_eint_md1.h>
#endif

struct ISDBT_INIT_INFO_T *hInit;

#define RING_BUFFER_SIZE	(188 * 320 * 8)

/* GPIO(RESET & INTRRUPT) Setting */
#define FC8300_NAME		"isdbt"

#if 1

#define GPIO_ISDBT_IRQ_NUM   	CUST_EINT_DTV_HOSTB_NUM
#define GPIO_ISDBT_IRQ   	GPIO_DTV_HOSTB
#define GPIO_ISDBT_PWR_EN 	GPIO_DTV_LDO_EN
#define GPIO_ISDBT_RST 		GPIO_DTV_IRESETB 
#define GPIO_DTV_FM_SWITCH 	GPIO_DTV_FM_ANT_SEL
#define GPIO_DTV_FM_SWITCH_1    GPIO_ANT_SW_PIN

#endif

struct ISDBT_OPEN_INFO_T hOpen_Val;
u8 static_ringbuffer[RING_BUFFER_SIZE];

enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;
static DEFINE_MUTEX(ringbuffer_lock);
static DEFINE_MUTEX(driver_mode_lock);
static DECLARE_WAIT_QUEUE_HEAD(isdbt_isr_wait);

#ifndef BBM_I2C_TSIF
static u8 isdbt_isr_sig=0;
static struct task_struct *isdbt_kthread;
#ifdef FEATURE_MTK
void isdbt_irq(void)
{
	//print_log(0, "isdbt_irq \n");

	isdbt_isr_sig = 1;

	wake_up_interruptible(&isdbt_isr_wait);
	
	return IRQ_HANDLED;
}
#else
static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
	isdbt_isr_sig = 1;
	wake_up_interruptible(&isdbt_isr_wait);
	return IRQ_HANDLED;
}
#endif
#endif
#ifdef FEATURE_MTK
int isdbt_hw_setting(void)
{
	print_log(0, "[FC8300] isdbt_hw_setting \n");
	
	mt_set_gpio_mode(GPIO_ISDBT_RST,  GPIO_DTV_IRESETB_M_GPIO);
	mt_set_gpio_dir(GPIO_ISDBT_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_ISDBT_RST, GPIO_OUT_ZERO);
#if 0
	mt_set_gpio_mode(GPIO_DTV_FM_SWITCH,  GPIO_DTV_FM_ANT_SEL_M_GPIO);
	mt_set_gpio_dir(GPIO_DTV_FM_SWITCH, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_DTV_FM_SWITCH, GPIO_OUT_ONE);
#endif

#if 1
	mt_set_gpio_mode(GPIO_ISDBT_IRQ, GPIO_DTV_HOSTB_M_EINT);
	mt_set_gpio_dir(GPIO_ISDBT_IRQ, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ISDBT_IRQ, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ISDBT_IRQ, GPIO_PULL_UP);
	//printk("[FC8300] isdbt_hw_setting state 2\n");
	mt_eint_registration(GPIO_ISDBT_IRQ_NUM, EINTF_TRIGGER_FALLING, isdbt_irq, 1);
	mt_eint_mask(GPIO_ISDBT_IRQ_NUM);	
        print_log(0,"irq_pin=%x,irq_num=%x\n",GPIO_ISDBT_IRQ,GPIO_ISDBT_IRQ_NUM);
#endif
////////////////////////////////
#if 0
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, isdbt_irq, 1); 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
//////////////////////////////////
	

	printk("[FC8300] isdbt_hw_setting exit\n");
	return 0;
}
#else
int isdbt_hw_setting(void)
{
	int err;
	print_log(0, "isdbt_hw_setting\n");

	err = gpio_request(GPIO_ISDBT_PWR_EN, "isdbt_en");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_en\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);

	err = gpio_request(GPIO_ISDBT_RST, "isdbt_rst");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_rst\n");
		goto gpio_isdbt_rst;
	}
	gpio_direction_output(GPIO_ISDBT_RST, 0);

#ifndef BBM_I2C_TSIF
	err = gpio_request(GPIO_ISDBT_IRQ, "isdbt_irq");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_irq\n");
		goto gpio_isdbt_rst;
	}

	gpio_direction_input(GPIO_ISDBT_IRQ);

	err = request_irq(GPIO_ISDBT_IRQ, isdbt_irq
		, IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8300_NAME, NULL);

	if (err < 0) {
		print_log(0,
			"isdbt_hw_setting: couldn't request gpio interrupt %d reason(%d)\n"
			, GPIO_ISDBT_IRQ, err);
	goto request_isdbt_irq;
	}
#endif

	return 0;
#ifndef BBM_I2C_TSIF
request_isdbt_irq:
	gpio_free(GPIO_ISDBT_IRQ);
#endif
gpio_isdbt_rst:
	gpio_free(GPIO_ISDBT_PWR_EN);
gpio_isdbt_en:
	return err;
}
#endif
/*POWER_ON & HW_RESET & INTERRUPT_CLEAR */
void isdbt_hw_init(void)
{
	mutex_lock(&driver_mode_lock);

	print_log(0, "isdbt_hw_init\n");
#ifdef FEATURE_MTK

	/*hwPowerOn(MT6323_POWER_LDO_VIO28, VOL_2800, "2V8_DTV");*/
	/*hwPowerOn(MT6323_POWER_LDO_VIO18, VOL_1800, "1V8_DTV");*/
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, FC8300_NAME);

	mt_set_gpio_mode(GPIO_ISDBT_RST,  GPIO_DTV_IRESETB_M_GPIO);
	mt_set_gpio_dir(GPIO_ISDBT_RST, GPIO_DIR_OUT);
	
	mt_set_gpio_mode(GPIO_ISDBT_PWR_EN,  GPIO_DTV_LDO_EN_M_GPIO);//GPIO_CMMB_RST_PIN_M_GPIO
	mt_set_gpio_dir(GPIO_ISDBT_PWR_EN, GPIO_DIR_OUT);	
    msWait(2);

    mt_set_gpio_out(GPIO_ISDBT_PWR_EN, GPIO_OUT_ONE);
#if 1
    mt_set_gpio_mode(GPIO_DTV_FM_SWITCH,  GPIO_DTV_FM_ANT_SEL_M_GPIO);
    mt_set_gpio_dir(GPIO_DTV_FM_SWITCH, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DTV_FM_SWITCH, GPIO_OUT_ONE);
    mt_set_gpio_mode(GPIO_DTV_FM_SWITCH_1,  GPIO_ANT_SW_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_DTV_FM_SWITCH_1, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DTV_FM_SWITCH_1, GPIO_OUT_ZERO);
#endif
    mt_set_gpio_out(GPIO_ISDBT_RST, GPIO_OUT_ONE);
    msWait(10);
    mt_set_gpio_out(GPIO_ISDBT_RST, GPIO_OUT_ZERO);
    msWait(15);
    mt_set_gpio_out(GPIO_ISDBT_RST, GPIO_OUT_ONE);
    msWait(15);
	
#else

	gpio_set_value(GPIO_ISDBT_RST, 0);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
	mdelay(5);
	gpio_set_value(GPIO_ISDBT_RST, 1);
	mdelay(5);
#endif
	driver_mode = ISDBT_POWERON;
	mutex_unlock(&driver_mode_lock);
}

/*POWER_OFF */
void isdbt_hw_deinit(void)
{
	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_deinit\n");

#ifdef FEATURE_MTK
    mt_set_gpio_out(GPIO_ISDBT_PWR_EN, GPIO_OUT_ZERO);
    mt_set_gpio_out(GPIO_ISDBT_RST, GPIO_OUT_ZERO);
	hwPowerDown(MT6323_POWER_LDO_VGP2,  FC8300_NAME);
	/*hwPowerDown(MT6323_POWER_LDO_VIO18, "1V8_DTV");*/
	/*hwPowerDown(MT6323_POWER_LDO_VIO28, "2V8_DTV");*/
#else
	gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
	gpio_set_value(GPIO_ISDBT_RST, 0);
#endif
	driver_mode = ISDBT_POWEROFF;
	mutex_unlock(&driver_mode_lock);
}

int data_callback(ulong hDevice, u8 bufid, u8 *data, int len)
{
	struct ISDBT_INIT_INFO_T *hInit;
	struct list_head *temp;
	hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	list_for_each(temp, &(hInit->hHead))
	{
		struct ISDBT_OPEN_INFO_T *hOpen;

		hOpen = list_entry(temp, struct ISDBT_OPEN_INFO_T, hList);

		if (hOpen->isdbttype == TS_TYPE) {
			mutex_lock(&ringbuffer_lock);
			if (fci_ringbuffer_free(&hOpen->RingBuffer) < len) {
				/*print_log(hDevice, "f"); */
				/* return 0 */;
				FCI_RINGBUFFER_SKIP(&hOpen->RingBuffer, len);
			}

			fci_ringbuffer_write(&hOpen->RingBuffer, data, len);

			wake_up_interruptible(&(hOpen->RingBuffer.queue));

			mutex_unlock(&ringbuffer_lock);
		}
	}

	return 0;
}


#ifndef BBM_I2C_TSIF
static int isdbt_thread(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	set_user_nice(current, -20);

	print_log(hInit, "isdbt_kthread enter\n");

	bbm_com_ts_callback_register((ulong)hInit, data_callback);

	while (1) {
	////////////////////////////////////////////
	#if 0
			mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
			set_current_state(TASK_INTERRUPTIBLE); 
			printk("isdbt_kthread --- waitting\n");
			wait_event_interruptible(waiter,isdbt_isr_sig!=0);				 
    #endif

	//////////////////////////////////////////////////
		wait_event_interruptible(isdbt_isr_wait, isdbt_isr_sig || kthread_should_stop());
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		mutex_lock(&driver_mode_lock);
		if (driver_mode == ISDBT_POWERON)
			bbm_com_isr(hInit);
		mutex_unlock(&driver_mode_lock);

		isdbt_isr_sig = 0;
		
		if (kthread_should_stop())
			break;
	}

	bbm_com_ts_callback_deregister();

	print_log(hInit, "isdbt_kthread exit\n");

	return 0;
}
#endif

const struct file_operations isdbt_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= isdbt_ioctl,
	.open		= isdbt_open,
	.read		= isdbt_read,
	.release	= isdbt_release,
};

static struct miscdevice fc8300_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FC8300_NAME,
	.fops = &isdbt_fops,
};

int isdbt_open(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt open\n");
	printk("fc8300 isdbt open\n");

	hOpen = &hOpen_Val;
	hOpen->buf = &static_ringbuffer[0];
	hOpen->isdbttype = 0;

	if (list_empty(&(hInit->hHead)))
		list_add(&(hOpen->hList), &(hInit->hHead));

	hOpen->hInit = (HANDLE *)hInit;

	if (hOpen->buf == NULL) {
		print_log(hInit, "ring buffer malloc error\n");
		return -ENOMEM;
	}

	fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);

	filp->private_data = hOpen;

	return 0;
}

ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s32 avail;
	s32 non_blocking = filp->f_flags & O_NONBLOCK;
	struct ISDBT_OPEN_INFO_T *hOpen
		= (struct ISDBT_OPEN_INFO_T *)filp->private_data;
	struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
	ssize_t len, read_len = 0;

	if (!cibuf->data || !count)	{
		/*print_log(hInit, " return 0\n"); */
		return 0;
	}

	if (non_blocking && (fci_ringbuffer_empty(cibuf)))	{
		/*print_log(hInit, "return EWOULDBLOCK\n"); */
		return -EWOULDBLOCK;
	}

	if (wait_event_interruptible(cibuf->queue,
		!fci_ringbuffer_empty(cibuf))) {
		print_log(hInit, "return ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	mutex_lock(&ringbuffer_lock);

	avail = fci_ringbuffer_avail(cibuf);

	if (count >= avail)
		len = avail;
	else
		len = count - (count % 188);

	read_len = fci_ringbuffer_read_user(cibuf, buf, len);

	mutex_unlock(&ringbuffer_lock);

	return read_len;
}

int isdbt_release(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt_release\n");
	isdbt_hw_deinit();

	hOpen = filp->private_data;

	hOpen->isdbttype = 0;
	if (!list_empty(&(hInit->hHead)))
		list_del(&(hOpen->hList));
	return 0;
}


#ifndef BBM_I2C_TSIF
void isdbt_isr_check(HANDLE hDevice)
{
	u8 isr_time = 0;

	bbm_com_write(hDevice, DIV_BROADCAST, BBM_BUF_ENABLE, 0x00);

	while (isr_time < 10) {
		if (!isdbt_isr_sig)
			break;

		msWait(10);
		isr_time++;
	}

}
#endif

long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	s32 res = BBM_NOK;
	s32 err = 0;
	s32 size = 0;
	struct ISDBT_OPEN_INFO_T *hOpen;

	struct ioctl_info info;
	//print_log(0, "[FC8300] entry IOCTL_ISDBT cmd =%x \n",cmd);
	if (_IOC_TYPE(cmd) != IOCTL_MAGIC)
		return -EINVAL;
	if (_IOC_NR(cmd) >= IOCTL_MAXNR)
		return -EINVAL;
	//print_log(0, "[FC8300] entry IOCTL_ISDBT _IOC_NR(cmd) =%d \n",_IOC_NR(cmd));
	hOpen = filp->private_data;

	size = _IOC_SIZE(cmd);

	switch (cmd) {
	case IOCTL_ISDBT_RESET:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_reset(hInit, (u16)info.buff[0]);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_RESET\n");
#endif
		break;
	case IOCTL_ISDBT_INIT:
		res = bbm_com_i2c_init(hInit, FCI_HPI_TYPE);
		res |= bbm_com_probe(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_INIT %d\n", BBM_XTAL_FREQ);
#endif
		if (res) {
			print_log(hInit, "FC8300 Initialize Fail\n");
			break;
		}
		res |= bbm_com_init(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_INIT res %d\n", res);
#endif
		break;
	case IOCTL_ISDBT_BYTE_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u8 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BYTE_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u8)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_WORD_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u16 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_WORD_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u16)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_LONG_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u32 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_LONG_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u32)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BULK_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_bulk_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u8 *)(&info.buff[3])
			, info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BULK_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BYTE_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u8)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BYTE_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_WORD_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u16)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_WORD_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_LONG_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0]
			, (u32)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_LONG_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_BULK_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_bulk_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u8 *)(&info.buff[3])
			, info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BULK_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_read(hInit, (u16)info.buff[3]
			, (u8)info.buff[0], (u8)info.buff[1]
			,  (u8 *)(&info.buff[4]), (u8)info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[3], (u16)info.buff[0], info.buff[4]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_TUNER_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_write(hInit, (u16)info.buff[3]
			, (u8)info.buff[0], (u8)info.buff[1]
			, (u8 *)(&info.buff[4]), (u8)info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[3], (u16)info.buff[0], info.buff[4]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_SET_FREQ:
		{
			u32 f_rf;
			u8 subch;
			err = copy_from_user((void *)&info, (void *)arg, size);

			f_rf = (u32)info.buff[0];
			subch = (u8)info.buff[1];
#ifndef BBM_I2C_TSIF
#ifdef FEATURE_MTK
			mt_eint_mask(GPIO_ISDBT_IRQ_NUM);
#endif
			isdbt_isr_check(hInit);
#endif
			res = bbm_com_tuner_set_freq(hInit
				, (u16)info.buff[2], f_rf, subch);

#ifdef FC8300_DEBUG
		print_log(0
		, "[FC8300] IOCTL_ISDBT_TUNER_SET_FREQ [0x%x][%d][0x%x]\n"
		, (u16)info.buff[2], f_rf, subch);
#endif

#ifndef BBM_I2C_TSIF
			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_flush(&hOpen->RingBuffer);
			mutex_unlock(&ringbuffer_lock);
			bbm_com_write(hInit
				, DIV_BROADCAST, BBM_BUF_ENABLE, 0x01);
#ifdef FEATURE_MTK
			mt_eint_unmask(GPIO_ISDBT_IRQ_NUM);//GPIO_ISDBT_IRQ_NUM CUST_EINT_ALS_NUM
#endif
#endif
		}
		break;
	case IOCTL_ISDBT_TUNER_SELECT:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_select(hInit, (u16)info.buff[2]
			, (u32)info.buff[0], (u32)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_SELECT [0x%x][%d][0x%x]\n"
		, (u16)info.buff[2], (u32)info.buff[0], (u32)info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_TS_START:
		hOpen->isdbttype = TS_TYPE;
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_START\n");
#endif
		break;
	case IOCTL_ISDBT_TS_STOP:
		hOpen->isdbttype = 0;
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_STOP\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_ON:
		isdbt_hw_init();
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_ON\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_OFF:
		isdbt_hw_deinit();
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_OFF\n");
#endif
		break;

	case IOCTL_ISDBT_SCAN_STATUS:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_scan_status(hInit, (u16)info.buff[0]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_SCAN_STATUS [0x%x]\n"
		, (u16)info.buff[0]);
#endif
		break;

	case IOCTL_ISDBT_TUNER_GET_RSSI:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_get_rssi(hInit
			, (u16)info.buff[0], (s32 *)&info.buff[1]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_GET_RSSI [0x%x][%d]\n"
		, (u16)info.buff[0], info.buff[1]);
#endif
		break;

	case IOCTL_ISDBT_DEINIT:
#ifdef FEATURE_MTK
		mt_eint_mask(GPIO_ISDBT_IRQ_NUM);
#endif
		res = bbm_com_deinit(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_DEINIT\n");
#endif
		break;

	default:
		print_log(hInit, "isdbt ioctl error!\n");
		res = BBM_NOK;
		break;
	}

	if (err < 0) {
		print_log(hInit, "copy to/from user fail : %d", err);
		res = BBM_NOK;
	}
	return res;
}

int isdbt_init(void)
{
	s32 res;

	print_log(hInit, "isdbt_init 20150715\n");

/////////////////////////////////////////
#if 0
	//res = hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800,mode_name);
	if(TRUE != hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, FC8300_NAME))
	{
		printk("[isdbt_init] Fail to enable digital power VGP2\n");
	}
	printk("[isdbt_init] enable digital power VGP2\n");
#endif
////////////////////////////////////////////////
	res = misc_register(&fc8300_misc_device);

	if (res < 0) {
		print_log(hInit, "isdbt init fail : %d\n", res);
		return res;
	}

	isdbt_hw_setting();

	hInit = kmalloc(sizeof(struct ISDBT_INIT_INFO_T), GFP_KERNEL);


#if defined(BBM_I2C_TSIF) || defined(BBM_I2C_SPI)
	res = bbm_com_hostif_select(hInit, BBM_I2C);
#else
	res = bbm_com_hostif_select(hInit, BBM_SPI);
#endif

	if (res)
		print_log(hInit, "isdbt host interface select fail!\n");

#ifndef BBM_I2C_TSIF
	if (!isdbt_kthread)	{
		print_log(hInit, "kthread run\n");
		isdbt_kthread = kthread_run(isdbt_thread
			, (void *)hInit, "isdbt_thread");
	}
#endif

	INIT_LIST_HEAD(&(hInit->hHead));
//	printk( "isdbt host interface select fail!\n");
	return 0;
}

void isdbt_exit(void)
{
	print_log(hInit, "isdbt isdbt_exit \n");
	s32 res;
	isdbt_hw_deinit();

#ifndef BBM_I2C_TSIF
	free_irq(GPIO_ISDBT_IRQ, NULL);
	gpio_free(GPIO_ISDBT_IRQ);
#endif

	gpio_free(GPIO_ISDBT_RST);
	gpio_free(GPIO_ISDBT_PWR_EN);

#ifndef BBM_I2C_TSIF
	if (isdbt_kthread)
		kthread_stop(isdbt_kthread);

	isdbt_kthread = NULL;
#endif

	bbm_com_hostif_deselect(hInit);
/////////////////////////////
	if(TRUE != hwPowerDown(MT6323_POWER_LDO_VGP2, FC8300_NAME)) {
		printk("[isdbt_exit] Fail to OFF digital power VGP2\n");
	}
//////////////////////////
	if (hInit != NULL)
		kfree(hInit);
	misc_deregister(&fc8300_misc_device);

}

module_init(isdbt_init);
module_exit(isdbt_exit);

MODULE_LICENSE("Dual BSD/GPL");

