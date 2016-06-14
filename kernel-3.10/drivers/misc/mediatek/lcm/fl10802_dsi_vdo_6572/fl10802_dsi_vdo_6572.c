#ifdef BUILD_LK
    #include <string.h>
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"


#if defined(BUILD_LK)
    #define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
    #define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)  // pixel
#define FRAME_HEIGHT (854)  // pixel

#define PHYSICAL_WIDTH  (62)  // mm
#define PHYSICAL_HEIGHT (110)  // mm

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFA    // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID       (0x10)
#define LCM_ID1       (0x80)
#define LCM_ID2       (0x1A)
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


//unsigned char vcom=80;

/*HSD*/
static struct LCM_setting_table lcm_initialization_setting[] = {
	

    //AP 
	{0xB9,	3,	{0xF1,0x08,0x01}},
	{0xB1,	4,	{0x22,0x1E,0x1E,0x87}},
	{0xB2,	1,	{0x22}},
	{0xB3,	8,	{0x01,0x00,0x06,0x06,0x18,0x13,0x39,0x35}},
  {0xBA,	17,	{0x31,0X00,0X44,0X25,0X91,0X0A,0X00,0X00,0XC1,0x00,
  	           0X00,0X00,0X0D,0X02,0X4f,0XB9,0XEE}},//NEW
	{0xE3,	5,	{0x03,0x03,0x03,0x03,0x00}},
	{0xB4,	1,	{0x00}},  	
	{0xB5,	2,	{0x0d,0x0d}},	
	{0xB6,	2,	{0x5B,0x5B}},//20151112 M 
	{0xB8,	2,	{0x64,0x28}}, //20-->28	
  {0xCC,	1,	{0x0C}},//0x00	RGB_PANEL		
	{0xBC,	1,	{0x47}},			
	{0xE9,	51,	{0x00,0x00,0x05,0x00,0x00,0x0A,0x90,0x12,0x30,0x00,
		           0x33,0x09,0x0A,0x90,0x37,0x00,0x03,0x00,0x00,0x00,
		           0x08,0x08,0x98,0xDD,0x20,0x64,0x02,0x88,0x88,0x88,
		           0x88,0x98,0xDD,0x31,0x75,0x13,0x88,0x88,0x88,0x88,
		           0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		           0x00}},        	           
  {0xEA,	22,	{0x94,0x00,0x00,0x00,0x89,0xDD,0x35,0x71,0x31,0x88,
  	           0x88,0x88,0x88,0x89,0xDD,0x24,0x60,0x20,0x88,0x88,
  	           0x88,0x88}},  
  	           	           
	{0xE0,	34,	{0x00,0x00,0x00,0x17,0x20,0x3F,0x29,0x3c,0x04,0x0E,0x13,0x18,0x1A,0x17,0x17,0x12,0x18,
			         0x00,0x00,0x00,0x17,0x20,0x3F,0x29,0x3c,0x04,0x0E,0x13,0x18,0x1A,0x17,0x17,0x12,0x18}},//AP
	
	
	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 120, {}},
	
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 10, {}},

	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#if 1
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    {0xE3,5,{0x03,0x03,0x03,0x03,0xc0}},
    {REGFLAG_DELAY, 1, {}},
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
        case REGFLAG_DELAY:
            MDELAY(table[i].count);
            break;
   /*     case 0xb6:
	        table[i].para_list[0] = vcom ;  
                table[i].para_list[1] = vcom ;
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		vcom+=1;
                break;
   */			
        case REGFLAG_END_OF_TABLE:
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);

            if (0/*cmd != 0xFF && cmd != 0x2C && cmd != 0x3C*/) {
                //#if defined(BUILD_UBOOT)
                //  printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
                //#endif
                while (read_reg(cmd) != table[i].para_list[0]);
            }
        }
    }
}
#endif


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{

    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // physical size
    params->physical_width = PHYSICAL_WIDTH;
    params->physical_height = PHYSICAL_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

	params->dsi.noncont_clock = 1 ;
	params->dsi.noncont_clock_period = 2 ;

#if defined(LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

#if defined(LCM_DSI_CMD_MODE)
    params->dsi.intermediat_buffer_num = 0; //because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count = 480 * 3;   //DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line = 854;
    params->dsi.compatibility_for_nvk = 0;  // this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
#else
	 
        params->dsi.word_count = 480 * 3;   //DSI CMD mode need set these two bellow params, different to 6577

	params->dsi.noncont_clock = 1;
	//params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 10;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 20;
	params->dsi.horizontal_backporch				= 70;//60
	params->dsi.horizontal_frontporch				= 70;//60
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#endif
    // Bit rate calculation
#if 1
    params->dsi.ssc_disable = 1;
    params->dsi.PLL_CLOCK = 208; //dsi clock customization: should config clock value directly
#else
        params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 1;
	params->dsi.fbk_sel = 1;
	params->dsi.fbk_div = 15;
#endif
}
#if 1

static void init_lcm_registers(void)
{
    unsigned int data_array[16];

    
    data_array[0]= 0x00043902;
    data_array[1]= 0x0108F1B9;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00053902;
    data_array[1]= 0x1E1E22B1;
    data_array[2]= 0x00000087;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]=0x22B21500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]=0x00093902;
    data_array[1]=0x06060001;
    data_array[2]=0x35391318;
    dsi_set_cmdq(data_array, 3, 1);
    
    
    data_array[0]=0x00123902;
    data_array[1]=0x440031BA;
    data_array[2]=0x000A9125;
    data_array[3]=0x0000E100;
    data_array[4]=0x4F020F00;
    data_array[5]=0x0000EEB9;
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0]=0x00063902;
    data_array[1]=0x030303E3;
    data_array[2]=0x00000003;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]=0x02B41500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x00A5A5b5;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x007777b6;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x002064b8;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00CC1500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]=0x47BC1500;
    dsi_set_cmdq(data_array, 1, 1);
    //Set GIP1
    data_array[0]=0x00343902;
    data_array[1]=0x050000E9;
    data_array[2]=0x12900A00;
    data_array[3]=0x09330030;
    data_array[4]=0x0037900A;
    data_array[5]=0x00000003;
    data_array[6]=0xDD980808;
    data_array[7]=0x88026420;
    data_array[8]=0x98888888;
    data_array[9]=0x137531dd;
    data_array[10]=0x88888888;
    data_array[11]=0x00000000;
    data_array[12]=0x00000000;
    data_array[13]=0x00000000;
    dsi_set_cmdq(data_array, 14, 1);
    
    //Set GIP2
    data_array[0]=0x00173902;
    data_array[1]=0x000094EA;
    data_array[2]=0x35DD8900;
    data_array[3]=0x88883171;
    data_array[4]=0xDD899988;
    data_array[5]=0x88206024;
    data_array[6]=0x00888888;
    dsi_set_cmdq(data_array, 7, 1);
     
    data_array[0]=0x00231500;
    dsi_set_cmdq(data_array, 1, 1); 

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    
    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
}
#endif

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(100);
    
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    
   // init_lcm_registers();
}


static void lcm_suspend(void)
{
    #if 1
	  push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
   
    SET_RESET_PIN(0);
    MDELAY(10);
    #else
    
	
    unsigned int data_array[16];

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    #endif
  
  //  dsi_set_cmdq(data_array, 1, 1);
  //  MDELAY(40);
}


static void lcm_resume(void)
{
    lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
    unsigned char x0_LSB = (x0 & 0xFF);
    unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
    unsigned char x1_LSB = (x1 & 0xFF);
    unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
    unsigned char y0_LSB = (y0 & 0xFF);
    unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
    unsigned char y1_LSB = (y1 & 0xFF);

    unsigned int data_array[16];


    data_array[0] = 0x00053902;
    data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
    data_array[2] = (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00053902;
    data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
    data_array[2] = (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


    if (level > 255)
        level = 255;

    data_array[0] = 0x00023902;
    data_array[1] = (0x51 | (level << 8));
    dsi_set_cmdq(data_array, 2, 1);
}


#if 0
static void lcm_setpwm(unsigned int divider)
{
    // TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
    // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
    // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
    unsigned int pwm_clk = 23706 / (1 << divider);


    return pwm_clk;
}
#endif


static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[4];
    unsigned int data_array[16];


    SET_RESET_PIN(1);           //NOTE:should reset LCM firstly
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(100);

    /*    
       data_array[0] = 0x00110500;        // Sleep Out
       dsi_set_cmdq(data_array, 1, 1);
       MDELAY(120);
     */

    //*************Enable CMD2 Page1  *******************//
    data_array[0]= 0x00043902;
    data_array[1]= 0x0108F1B9;   
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);

    data_array[0] = 0x00043700; // read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    read_reg_v2(0x04, buffer, 4);
    id = buffer[0];             //we only need ID
#ifdef BUILD_LK
	printf("FL10802 id = 0x%08x id1=%x id2=%x \n",  id,buffer[2],buffer[3]);
#else
	//printk("FL11281 id = 0x%08x  id1=%x \n",id,id1);
#endif

        return (LCM_ID == id && LCM_ID1==buffer[1]) ? 1 : 0;
}

void lcm_read_fb(unsigned char *buffer)
{
	  unsigned int array[2];

   array[0] = 0x000A3700;// read size
   dsi_set_cmdq(array, 1, 1);
   
   read_reg_v2(0x2E,buffer,10);
   read_reg_v2(0x3E,buffer+10,10);
   read_reg_v2(0x3E,buffer+10*2,10);
   read_reg_v2(0x3E,buffer+10*3,10);
   read_reg_v2(0x3E,buffer+10*4,10);
   read_reg_v2(0x3E,buffer+10*5,10);
}

static unsigned int lcm_esd_check(void)
{
   unsigned int id = 0;
    unsigned char buffer[4];
    unsigned int data_array[16];

    /*    
       data_array[0] = 0x00110500;        // Sleep Out
       dsi_set_cmdq(data_array, 1, 1);
       MDELAY(120);
     */

    //*************Enable CMD2 Page1  *******************//
    /*data_array[0]= 0x00043902;
    data_array[1]= 0x0108F1B9;   
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);
*/
   /* data_array[0] = 0x00013700; // read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
*/
    read_reg_v2(0x0a, buffer, 1);
   // id = buffer[0];             //we only need ID
//#ifndef BUILD_LK
//	printk("FL10802 esd check 0x0a:  %x \n",buffer[0]);
//#endif
     return (buffer[0] == 0x9c ) ? 0 : 1;
}
 static unsigned int  lcm_esd_recover(void)
{
	lcm_init();
        return 1;
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER fl10802_dsi_vdo_6572_drv = {
    .name = "fl10802_dsi_vdo_6572",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    //.set_backlight = lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .compare_id = lcm_compare_id,
    .esd_check = lcm_esd_check,
    .esd_recover= lcm_esd_recover,
    //.update = lcm_update,
    //.read_fb = lcm_read_fb,
};
