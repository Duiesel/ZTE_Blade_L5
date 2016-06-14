/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2013. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
// pixel
#define FRAME_WIDTH  			(480)
#define FRAME_HEIGHT 			(854)
// physical dimension
#define PHYSICAL_WIDTH        (68)
#define PHYSICAL_HIGHT         (121)

#define REGFLAG_DELAY                                       0xFE
#define REGFLAG_END_OF_TABLE                                0x00   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE                                    0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)            lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define   LCM_DSI_CMD_MODE                          0

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

#if 1 //zgd ili9806e

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
	{0x08,1,{0x10}},
	{0x21,1,{0x01}},
	//{0x22,1,{0x01}},
	{0x30,1,{0x01}},
	{0x31,1,{0x02}},
	{0x40,1,{0x15}},
	{0x41,1,{0x55}},
	{0x42,1,{0x02}},
	{0x43,1,{0x09}},
	{0x44,1,{0x06}},
	//{0x46,1,{0x55}},
	{0x50,1,{0x78}},
	{0x51,1,{0x78}},
	{0x52,1,{0x00}},
	{0x53,1,{0x44}},
	{0x57,1,{0x50}},
	
	{0x60,1,{0x07}},
	{0x61,1,{0x00}},
	{0x62,1,{0x07}},
	{0x63,1,{0x00}},

	//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
	{0xa0,1,{0x00}},
	{0xa1,1,{0x08}},
	{0xa2,1,{0x13}},
	{0xa3,1,{0x11}},
	{0xa4,1,{0x0A}},
	{0xa5,1,{0x18}},
	{0xa6,1,{0x04}},
	{0xa7,1,{0x05}},
	{0xa8,1,{0x07}},
	{0xa9,1,{0x0C}},
	{0xaa,1,{0x0C}},
	{0xab,1,{0x06}},
	{0xac,1,{0x0D}},
	{0xad,1,{0x29}},
	{0xae,1,{0x24}},
	{0xaf,1,{0x00}},

	///==============Nagitive
	{0xc0,1,{0x00}},
	{0xc1,1,{0x08}},
	{0xc2,1,{0x13}},
	{0xc3,1,{0x11}},
	{0xc4,1,{0x0B}},
	{0xc5,1,{0x17}},
	{0xc6,1,{0x0E}},
	{0xc7,1,{0x0A}},
	{0xc8,1,{0x02}},
	{0xc9,1,{0x07}},
	{0xca,1,{0x02}},
	{0xcb,1,{0x05}},
	{0xcc,1,{0x0C}},
	{0xcd,1,{0x28}},
	{0xce,1,{0x24}},
	{0xcf,1,{0x00}},

	// Change to Page 6
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},
	{0x00, 1,{0x21}},
	{0x01, 1,{0x06}},
	{0x02, 1,{0xA0}},
	{0x03, 1,{0x02}},
	{0x04, 1,{0x01}},
	{0x05, 1,{0x01}},
	{0x06, 1,{0x80}},
	{0x07, 1,{0x03}},
	{0x08, 1,{0x06}},
	{0x09, 1,{0x80}},
	{0x0a, 1,{0x00}},
	{0x0b, 1,{0x00}},
	{0x0c, 1,{0x20}},
	{0x0d, 1,{0x20}},
	{0x0e, 1,{0x09}},
	{0x0f, 1,{0x00}},
	{0x10, 1,{0xfF}},
	{0x11, 1,{0xE0}},
	{0x12, 1,{0x00}},
	{0x13, 1,{0x00}},
	{0x14, 1,{0x00}},
	{0x15, 1,{0xc0}},
	{0x16, 1,{0x08}},
	{0x17, 1,{0x00}},
	{0x18, 1,{0x00}},
	{0x19, 1,{0x00}},
	{0x1a, 1,{0x00}},
	{0x1b, 1,{0x00}},
	{0x1c, 1,{0x00}},
	{0x1d, 1,{0x00}},

	{0x20, 1,{0x01}},
	{0x21, 1,{0x23}},
	{0x22, 1,{0x45}},
	{0x23, 1,{0x67}},
	{0x24, 1,{0x01}},
	{0x25, 1,{0x23}},
	{0x26, 1,{0x45}},
	{0x27, 1,{0x67}},

	{0x30, 1,{0x12}},
	{0x31, 1,{0x22}},
	{0x32, 1,{0x22}},
	{0x33, 1,{0x22}},
	{0x34, 1,{0x87}},
	{0x35, 1,{0x96}},
	{0x36, 1,{0xAA}},
	{0x37, 1,{0xdB}},
	{0x38, 1,{0xCC}},
	{0x39, 1,{0xBD}},
	{0x3a, 1,{0x78}},
	{0x3b, 1,{0x69}},
	{0x3c, 1,{0x22}},
	{0x3d, 1,{0x22}},
	{0x3e, 1,{0x22}},
	{0x3f, 1,{0x22}},
	{0x40, 1,{0x22}},
	{0x52, 1,{0x10}},
	{0x53, 1,{0x10}},
	//{0x58, 1,{0xA7}},

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7
	{0x17,1,{0x22}},
	{0x02,1,{0x77}},
	{0xe1,1,{0x79}},
	{0x00,1,{0x11}},
  {0x26,1,{0xB2}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     //
	{0x36,1,{0x03}},	//add	
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29,1,{0x00}},             // Display On
	{REGFLAG_DELAY,50,{}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}


#endif
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xFF,  3,  {0x80, 0x09, 0x01}},
	//{REGFLAG_DELAY, 10, {}},

	{0x00,  1,  {0x80}},
	//{REGFLAG_DELAY, 10, {}},

	{0xFF,  2,  {0x80,0x09}},
	//{REGFLAG_DELAY, 10, {}},

	{0x00,  1,  {0x03}},
	//{REGFLAG_DELAY, 10, {}},

	{0xFF,  1,  {0x01}},
	//{REGFLAG_DELAY, 10, {}},

	// Sleep Mode On
	// {0xC3, 1, {0xFF}},

	//{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	//{0xe3,5,{0x05,0x05,0x01,0x01,0xc0}},//fang zhi LC jihua

	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
            
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
                
            case REGFLAG_END_OF_TABLE :
                break;
                
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                MDELAY(2);
        }
    }    
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 6;// 4;2
	params->dsi.vertical_backporch					= 14;// 8;2
	params->dsi.vertical_frontporch					= 20;// 8;2
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 50;  //200
	params->dsi.horizontal_frontporch				= 50;  //200
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK=210;//215

	/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
	params->dsi.lcm_int_te_monitor = FALSE; 
	params->dsi.lcm_int_te_period = 1; // Unit : frames 

	// Need longer FP for more opportunity to do int. TE monitor applicably. 
	if(params->dsi.lcm_int_te_monitor) 
		params->dsi.vertical_frontporch *= 2; 

	// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
	params->dsi.lcm_ext_te_monitor = FALSE; 
	// Non-continuous clock 
	params->dsi.noncont_clock = TRUE; 
	params->dsi.noncont_clock_period = 2; // Unit : frames
}

//add by hyde for debug
static void hyde_degug()
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(200);
    
    array[0] = 0x00053700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xa1, buffer, 5);

    id_high = buffer[2];
    id_low = buffer[3];
    id = (id_high<<8) | id_low;

    #if defined(BUILD_LK)
        printf("OTM8018B uboot hyde compare_id%s \n", __func__);
        printf("%s id = 0x%08x \n", __func__, id);
        printf("hyde id is %08x \n",id);
    #else
        printk("OTM8018B kernel hyde compare_id%s \n", __func__);
        printk("%s id = 0x%08x \n", __func__, id);
        printk("hyde id is %08x \n",id);
    #endif

}

//add by hyde for debug
static void lcm_init(void)
{
    //void hyde_degug();
    SET_RESET_PIN(1);
    MDELAY(10); 
    SET_RESET_PIN(0);
    MDELAY(10); 
    
    SET_RESET_PIN(1);
    MDELAY(120); 

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);   
}

static void lcm_suspend(void)
{
/*
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(150);//Must > 120ms
*/    
    push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
   // SET_RESET_PIN(0);
    //MDELAY(10);//Must > 10ms
	
}

static unsigned int lcm_compare_id(void);
static void lcm_resume(void)
{
//lcm_compare_id();
	//lcm_init();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}
#endif

static unsigned int lcm_compare_id(void)
{
#if 0
	unsigned int id = 0; 
	unsigned char buffer[3]; 
	unsigned int data_array[16]; 


	SET_RESET_PIN(1); 
	MDELAY(10); 
	SET_RESET_PIN(0); 
	MDELAY(50); 
	SET_RESET_PIN(1); 
	MDELAY(120); 

	// Set Maximum return byte = 1 
	data_array[0] = 0x00033700;// read id return two byte,version and id 
	dsi_set_cmdq(data_array, 1, 1); 
	//MDELAY(10); 


	read_reg_v2(0x04, buffer, 3);  //DA DB DC
	id = buffer[0]; //we only need ID   //0x10 
	id<<=8; 
	id|= buffer[1]; //we test buffer 1  //0x80 

#if defined(BUILD_LK) 
	printf("[fl10801->lcm_esd_check] %s buffer[0] = %x; buffer[1]= %x; buffer[2]= %x;\n", __func__,buffer[0],buffer[1],buffer[2]); 
#elif defined(BUILD_UBOOT) 
	printf("[fl10801->lcm_esd_check] %s buffer[0] = %x; buffer[1]= %x; buffer[2]= %x;\n", __func__,buffer[0],buffer[1],buffer[2]); 
#else 
	printk("[fl10801->lcm_esd_check] %s buffer[0] = %x; buffer[1]= %x; buffer[2]= %x;\n", __func__,buffer[0],buffer[1],buffer[2]); 
#endif 

#ifdef BUILD_LK 
	printf("[lcm_compare_id] fl10801_ic:lcd id 0x%x\n",id); 
#else 
	printk("[lcm_compare_id] fl10801_ic:lcd id 0x%x\n",id); 
#endif 

	return (0x1080 == id)? 1:0; 
        //return 1;
#endif
#if 1
	//add by chentao begin
	int lcm_adc = 0;
	int data[4];
	int adc0 = 0;
	int adc1 = 0;
	int adcvoltage = 0;
	//add by chentao end

	unsigned int id,id1=0;
	unsigned char buffer[3];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	MDELAY(5); 
	SET_RESET_PIN(0);
	MDELAY(50); 
	SET_RESET_PIN(1);
	MDELAY(120); 

	array[0]=0x00063902;
	array[1]=0x0698FFFF;// page enable
	array[2]=0x00000104;
	dsi_set_cmdq(array, 3, 1);

	array[0]=0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x00, buffer, 2);
//	read_reg_v2(0x01, &buffer[0], 1);
//	read_reg_v2(0x02, &buffer[1], 1);
	id  =  buffer[1]; 
	id1 =  buffer[0]; 

 	lcm_adc = IMM_GetOneChannelValue(12,data,0); //Channel 2
 	adcvoltage = data[0] *1000 + data[1] *10;

#ifdef BUILD_LK
	printf("ili9806e id1 = 0x%x\r\n",id1);
	printf("ili9806e id = 0x%x\r\n",id);
	printf("txd ili9806e adc0 = %d\n",data[0]);
	printf("txd ili9806e adc1 = %d\n",data[1]);
	printf ("txd ili9806e read the adc  Voltage=%d  mv\n ",adcvoltage );
#else 
	printk("ili9806e id1 = 0x%x\r\n",id1);
	printk("ili9806e id = 0x%x\r\n",id);
	printk("txd ili9806e adc0 = %d\n",data[0]);
	printk("txd ili9806e adc1 = %d\n",data[1]);
	printk ("txd ili9806e read the adc  Voltage=%d  mv\n ",adcvoltage );
#endif
 //	return( (0x98 == id)&&((adcvoltage <1000) && (adcvoltage >=800)))? 1:0;	//zgd votage 0.9v
	return (0x98 == id) ?1:0;
#endif	 
}


static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	unsigned int id ,id1,id2,id3,id4,id5,id6,id7 = 0;  
	unsigned char buffer[6];   
	unsigned int data_array[16];   
	static int lcm_id;  
	UDELAY(600);      
	data_array[0] = 0x00013700;// read id return two byte,version and id  
	dsi_set_cmdq(data_array, 1, 1);  
	//MDELAY(10);
	read_reg_v2(0x0A, buffer, 1);    // A1  
	id = buffer[0]; //we only need ID     
	//printk("ghj ########## 9806e_gp lcd_id=%x,\r\n",id);     
	if(id ==0x9c)     
	{    
	return 0;   
	}      
	else      
	{         
	return 1; //TRUE     
 
	}
 #endif

}


static unsigned int lcm_esd_recover(void)
{
    lcm_init();
    lcm_resume();

    return TRUE;
}

LCM_DRIVER ili9806e_dsi_vdo_fwvga_drv = {
	.name = "ili9806e_dsi_vdo_fwvga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
//    .esd_check = lcm_esd_check,
//    .esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
