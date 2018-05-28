/* embARC HAL */
#include "embARC.h"
#include "embARC_debug.h"

/* middleware level*/
#include "u8g.h"

#include <stdlib.h>

#define SPI_MASTER_ID		0

#define SPI_SLAVE_FREQ		2000000

DEV_SPI *spi_master;

#define PMOD_ALS_SELECT      EMSK_SPI_LINE_0
// SPI端口参数
// EMSK_SPI_LINE_0:PMOD6 Pin 1 
// EMSK_SPI_LINE_1:PMOD5 Pin 1 or PMOD6 Pin 7 
// EMSK_SPI_LINE_2:PMOD6 Pin 8

#define SPI_SLV_DFS		8

#define BUF_LEN	2

// GPOI地址
#define ADDR_BTN_MENU 0x40000
#define ADDR_BTN_BACK 0x80000
#define ADDR_BTN_UP   0x20000
#define ADDR_BTN_DOWN 0x40000

#define ADDR_MOT_L   0x100000
#define ADDR_MOT_R   0x100000
#define ADDR_MOT_EN  0X200000

uint32_t rx_data[BUF_LEN];
static int brit_data = 0;
	
static volatile int rx_cb_done = 0;

//菜单显示索引
char func_index = 8;

// 时钟相关变量
volatile static int ss = 0;
static int mm;
static int hh;
static int day;
static int month;
static int year;
static int clk_hh;
static int clk_mm;
char date_y[10] = " ";
char date_m[10] = " ";
char date_d[10] = " ";
char time_h[10] = " ";
char time_m[10] = " ";
char time_s[10] = " ";
int Feb_day;

//模式相关变量
int func_mode = 1;

//亮度相关变量
char brit[10]   = " ";
int para_sensor = 1;

// OLED 的 reset 信号端口
static DEV_GPIO_PTR port_rst;  //PORT A[17] 0x40000(1<<17),J3 Pin7

// 按键的端口
static DEV_GPIO_PTR btn_menu; //PORT A[18] 0x40000(1<<18),J3 Pin8
static DEV_GPIO_PTR btn_back; //PORT A[19] 0x80000(1<<19),J3 Pin10
static DEV_GPIO_PTR btn_up;   //PORT C[17] 0x20000(1<<17),J3 Pin2
static DEV_GPIO_PTR btn_down; //PORT C[18] 0x40000(1<<18),J3 Pin3 
// 减速电机端口
static DEV_GPIO_PTR motor_l;  //PORT C[20],J4 Pin1
static DEV_GPIO_PTR motor_r;  //PORT A[20],J4 Pin7
static DEV_GPIO_PTR motor_en;  //PORT C[21],J4 Pin2

// 按键读取存储变量
uint32_t key_menu; // 菜单按钮
uint32_t key_back; // 确定按钮
uint32_t key_up;   // ”<-“按钮
uint32_t key_down; // ”->“按钮

u8g_t u8g;

// 延时函数
static void delay(uint32_t z)
{
	volatile uint32_t x, y;
	for (x = 36 ;x > 0 ;x --)
		for (y = z;y > 0;y --);
}

// SPI初始化
void slvdev_init(uint32_t freq)
{
	spi_master = spi_get_dev(SPI_MASTER_ID); /* Get SPI device, DW_SPI_0_ID/DW_SPI_1_ID */

	spi_master->spi_open(DEV_MASTER_MODE, freq); /* Open SPI device in master mode */

	spi_master->spi_control(SPI_CMD_SET_CLK_MODE, CONV2VOID(SPI_CLK_MODE_3)); /* Set SPI clock mode */
	spi_master->spi_control(SPI_CMD_SET_DFS, CONV2VOID(PMOD_ALS_SELECT)); /* Set SPI data frame size */
}

// 读取PMODALS数据
void ALS_read(uint8_t *buf, uint8_t len)
{
	spi_master->spi_control(SPI_CMD_MST_SEL_DEV, CONV2VOID(PMOD_ALS_SELECT)); /* Select SPI slave device */
	spi_master->spi_read(buf, len); /* Read operation by polling */
	spi_master->spi_control(SPI_CMD_MST_DSEL_DEV, CONV2VOID(PMOD_ALS_SELECT)); /* Deselect SPI slave device */
}

static void timer0_isr(void *ptr);

// 计时器初始化
void timer_initial()
{	
	uint32_t val;
	if (timer_present(TIMER_0)) {
		EMBARC_PRINTF(" - [INFO] timer 0 is present\n");
		timer_current(TIMER_0, &val);
		EMBARC_PRINTF(" - cnt:%d\n", val);
		timer_stop(TIMER_0); /* Stop it first since it might be enabled before */
		int_handler_install(INTNO_TIMER0, timer0_isr);
		int_enable(INTNO_TIMER0);
	}
	
	if (timer_present(TIMER_0)) {
		timer_start(TIMER_0, TIMER_CTRL_IE, BOARD_CPU_CLOCK);
	}
	ss    = 0;
	mm    = 0;
    hh    = 0;
    day   = 1;
    month = 1;
    year  = 2018;
}

// 电机端口初始化
void motor_initial()
{
	motor_l  =  gpio_get_dev(DW_GPIO_PORT_C);
	motor_l -> gpio_close();
	motor_l -> gpio_open(ADDR_MOT_L);
	
	motor_r  =  gpio_get_dev(DW_GPIO_PORT_A);
	motor_r -> gpio_close();
	motor_r -> gpio_open(ADDR_MOT_R);
	
	motor_en =  gpio_get_dev(DW_GPIO_PORT_C);
	motor_en -> gpio_close();
	motor_en -> gpio_open(ADDR_MOT_EN);
	
	motor_l  -> gpio_write(0x0,ADDR_MOT_L);
	motor_r  -> gpio_write(0x0,ADDR_MOT_R);
	motor_en -> gpio_write(0x0,ADDR_MOT_EN);
	
	motor_l  -> gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)ADDR_MOT_L);
	motor_r  -> gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)ADDR_MOT_R);
	motor_en  -> gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)ADDR_MOT_EN);
	
	EMBARC_PRINTF(" - [INFO] MOTOR PORT INITIALIZED. \n");
}
// 电机控制函数
void motor_enable(int t)  // 电机使能函数，t=1,enable;t=0,disable
{
	switch(t)
	{
		case 0:
		{
			motor_en -> gpio_write(0x0,ADDR_MOT_EN);break;
		}
		case 1:
		{
			motor_en -> gpio_write(ADDR_MOT_EN,ADDR_MOT_EN);break;
		}
	}
}
void motor_forward() //电机正转控制函数
{
	motor_l -> gpio_write(ADDR_MOT_L,ADDR_MOT_L);
	motor_r -> gpio_write(0x0,ADDR_MOT_R);
}
void motor_backward() //电机反转控制函数
{
	motor_l -> gpio_write(0x0,ADDR_MOT_L);
	motor_r -> gpio_write(ADDR_MOT_R,ADDR_MOT_R);
}
void motor_stop() //电机停止函数
{
	motor_l -> gpio_write(0x0,ADDR_MOT_L);
	motor_r -> gpio_write(0x0,ADDR_MOT_R);
}

// 按键端口初始化
void btn_initial()
{
	btn_menu =  gpio_get_dev(DW_GPIO_PORT_A);
	btn_menu -> gpio_close();
	btn_menu -> gpio_open(ADDR_BTN_MENU);
	
	btn_back =  gpio_get_dev(DW_GPIO_PORT_A);
	btn_back -> gpio_close();
	btn_back -> gpio_open(ADDR_BTN_BACK);
	
	btn_up =  gpio_get_dev(DW_GPIO_PORT_C);
	btn_up -> gpio_close();
	btn_up -> gpio_open(ADDR_BTN_UP);
	
	btn_down =  gpio_get_dev(DW_GPIO_PORT_C);
	btn_down -> gpio_close();
	btn_down -> gpio_open(ADDR_BTN_DOWN);
	EMBARC_PRINTF(" - [INFO] BTN PORT INITIALIZED \n");
	
	btn_menu  -> gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)ADDR_BTN_MENU);
	btn_back -> gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)ADDR_BTN_BACK);
	btn_up   -> gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)ADDR_BTN_UP);
	btn_down -> gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)ADDR_BTN_DOWN);
	EMBARC_PRINTF(" - [INFO] BTN PORT SETTING COMPLETED\n");
}
void read_menu();
void read_back();
void read_up();
void read_down();

void u8g_prepare(void) {
	u8g_SetFont(&u8g, u8g_font_unifont);		
	u8g_SetFontRefHeightExtendedText(&u8g);		
	u8g_SetDefaultForegroundColor(&u8g);	
	u8g_SetFontPosTop(&u8g);			
}

// OLED初始化
void OLED_initial()
{
	port_rst = gpio_get_dev(DW_GPIO_PORT_A);
	port_rst -> gpio_close();
	port_rst -> gpio_open(0x20000);
	port_rst -> gpio_write(0x0,0x20000);
	board_delay_ms(500, 1);
	port_rst -> gpio_write(0x20000,0x20000);
	
}

// 屏幕数据标题显示函数
void draw_title()
{	
	u8g_DrawStr(&u8g, 0, 0, "Date:");
	u8g_DrawStr(&u8g, 0, 14, "Time:");
	u8g_DrawStr(&u8g, 0, 28, "Light:");
	u8g_DrawStr(&u8g, 0, 42, "Status:");
}

//数据显示预处理函数
void proc_data()
{
	sprintf(date_y,"%4d",year);
	sprintf(date_m,"%02d",month);
	sprintf(date_d,"%02d",day);
	sprintf(time_h,"%02d",hh);
	sprintf(time_m,"%02d",mm);
	sprintf(time_s,"%02d",ss);
	sprintf(brit,"%d",brit_data);
}
// 屏幕数据显示函数
void draw_data()
{
	u8g_DrawStr(&u8g, 40, 0, date_y);
	u8g_DrawStr(&u8g, 73, 0, date_m);
	u8g_DrawStr(&u8g, 95, 0, date_d);
	u8g_DrawStr(&u8g, 40, 14, time_h);
	u8g_DrawStr(&u8g, 57, 14, time_m);
	u8g_DrawStr(&u8g, 82, 14, time_s);
	u8g_DrawStr(&u8g, 50, 28,  brit);
	u8g_DrawStr(&u8g, 60, 42, "Unknown");

}

// 数据处理和显示函数
void display_data(uint8_t *buf, uint8_t len)
{	
	uint8_t buf_MSB;
	uint8_t buf_LSB;
	buf_MSB = buf[0] << 3;
	buf_LSB = buf[1] >> 5;
	brit_data = buf_MSB + buf_LSB;
	EMBARC_PRINTF("   BRIGHTNESS DATA:");
	EMBARC_PRINTF("%d\t", brit_data);
	EMBARC_PRINTF("\r\n");	
	EMBARC_PRINTF("   BRIGHTNESS MSB DATA:");
	EMBARC_PRINTF("%b\t", buf_MSB);	
	EMBARC_PRINTF("\r\n");
	EMBARC_PRINTF("   BRIGHTNESS LSB DATA:");
	EMBARC_PRINTF("%b\t", buf_LSB);	
	EMBARC_PRINTF("\r\n");
	
	EMBARC_PRINTF(" - Sending data to OLED.\r\n");
	u8g_FirstPage(&u8g);
	do {
    draw_title();
	proc_data();
	draw_data();
	} while (u8g_NextPage(&u8g)); 
}



void sys_initial()
{
	OLED_initial(); //reset the OLED
	EMBARC_PRINTF("- [INFO] OLED reset completed.\r\n");
	
	timer_initial();
	EMBARC_PRINTF("- [INFO] timer0 initialized.\r\n");
	
	u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_2x_i2c, U8G_COM_SSD_I2C); //create a new interface to a graphics display
	u8g_Begin(&u8g); //reset display and put it into default state
	u8g_prepare();
	
	slvdev_init(SPI_SLAVE_FREQ);
	EMBARC_PRINTF("- [INFO] slvdev initialized.\r\n");
	btn_initial();
	motor_initial();
	
}

// 菜单显示功能相关
void (*current_operation_index)();

typedef struct
    {
        unsigned char current;
        unsigned char up;//向上翻索引号
        unsigned char down;//向下翻索引号
        unsigned char enter;//确认索引号
		unsigned char back;//返回索引号
        void (*current_operation)();
    } key_table;
	
//菜单执行函数声明
uint8_t get_back; //菜单返回所需变量

void menu_screen_p1(int line)
{	
	u8g_FirstPage(&u8g);
	do {

		u8g_DrawStr(&u8g, 40, 0, "*MENU*");
		u8g_DrawStr(&u8g, 2, 16, "Manual Control");
		u8g_DrawStr(&u8g, 2, 32, "Settings");
		u8g_DrawStr(&u8g, 2, 48, "Main Page");
		u8g_DrawFrame(&u8g,0,16*line-2,120,18);
	} while (u8g_NextPage(&u8g));
}
void menu_screen_p2(int line)
{	
	u8g_FirstPage(&u8g);
	do {

        u8g_DrawStr(&u8g, 2, 2, "Set Alarm");
		u8g_DrawStr(&u8g, 2, 18, "Set Time&Date");
		u8g_DrawStr(&u8g, 2, 34, "Set Mode");
		u8g_DrawStr(&u8g, 2, 50, "Sensor Celi");
		u8g_DrawFrame(&u8g,0,16*line,120,18);
	} while (u8g_NextPage(&u8g)); 
}

void func_1();
void func_2();
void func_3();
void func_4();
void func_5();
void func_6();
void func_7();
void func_8();
void func_9();
void func_10();
void func_11();
void func_12();
void func_13();

// 菜单索引结构体
const key_table table[]=
{
	{0,2,1,7,0,(*func_1)},//Page1 ,[1.Manual Control] 2.Settings  3.Main Page
	{1,0,2,3,1,(*func_2)},//Page1 , 1.Manual Control [2.Settings] 3.Main Page
	{2,1,0,8,0,(*func_3)},//Page1 , 1.Manual Control  2.Settings [3.Main Page]
	{3,6,4,9,1,(*func_4)},//Page2 ,[1.Set Alarm] 2.Set T&D   3.Set Mode  4.Sensor Celibration
	{4,3,5,10,1,(*func_5)},//Page2 , 1.Set Alarm [2.Set T&D]  3.Set Mode  4.Sensor Celibration
	{5,4,6,11,1,(*func_6)},//Page2 , 1.Set Alarm  2.Set T&D [3.Set Mode] 4.Sensor Celibration
	{6,5,3,12,1,(*func_7)},//Page2 , 1.Set Alarm  2.Set T&D  3.Set Mode [4.Sensor Celibration]
	
	{7,7,7,0,0,(*func_8)},// Manual Control
	{8,8,8,2,2,(*func_9)},// Main Page
	{9,9,9,3,3,(*func_10)},// Set Alarm
	{10,10,10,4,4,(*func_11)},// Set T&D
	{11,11,11,5,5,(*func_12)},// Set Mode
	{12,12,12,6,6,(*func_13)},// Sensor Celibration
	
};

// 定时功能函数
void alarm_clock(int ring_h,int ring_m,int pmw,int dir)
{
	/*
	static int clk_hh;
	static int clk_mm;
	*/
	int n = 100;
	if((ring_h == hh)&&(ring_m == mm))
	{
		
		if(dir == 1) motor_forward();
		if(dir == 0) motor_backward();
		while(n >= 0)
		{
			n--;
			motor_enable(0); // 可输出脉冲实现调速
			delay(pmw);
			motor_enable(1);
			delay(pmw);
		}
	}
}
// 主函数
int main(void)
{
	uint32_t n = 0;
	cpu_lock();	
	board_init();	
	cpu_unlock();	
	
	sys_initial();
	
	/*
	while (1) {
		EMBARC_PRINTF("\r\n>> TESTING loop %d <<\r\n", loop_cnt++);
		spi_test();
		delay(500);
	}
	*/
	func_0();	
	while(1)
	{	n++;

		delay(10);

		read_menu();
		read_back();
		read_up();
		read_down();
		if((key_menu==1)||(key_back==1)||(key_up==1)||(key_down==1))
			{
                delay(10);//消抖
                if(key_up==1)
                {
					func_index=table[func_index].up;
					EMBARC_PRINTF("- [INFO] PRESS UP.\r\n");					//向上翻
					while(key_up)
					{
						read_up();
					}//松手检测
                }
								 
                if(key_down==1)
                {
					func_index=table[func_index].down;
					EMBARC_PRINTF("- [INFO] PRESS DOWN.\r\n");//向下翻
					while(key_down)
					{
						read_down();
					}
                }
								 
                if(key_menu==1)
                {
					func_index=table[func_index].enter;    //确认
					EMBARC_PRINTF("- [INFO] PRESS MENU.\r\n");
					while(key_menu)
						{
							read_menu();
						}
                }
									 
                if(key_back==1)
                {
					func_index=table[func_index].back;    //返回
					EMBARC_PRINTF("- [INFO] PRESS BACK.\r\n");
					while(key_back)
						{
							read_back();
						}
                }
									 
                current_operation_index=table[func_index].current_operation;
               (*current_operation_index)();//执行当前操作函数
            }                                    

	}		

	
error_exit:
	return E_SYS;
}

// 计时器中断函数
static void timer0_isr(void *ptr)
{	
	timer_int_clear(TIMER_0);
	ss++;

	if(ss == 60)
	{
		ss = 0;
		mm++;

		if(mm == 60)
		{
			mm = 0;
			hh++;
			if(hh == 24)
			{
				hh = 0;
				day++;
				switch(month)
				{
					case 1:
						if(day == 32) {month++;day = 1;}
					case 2:
						if(day == Feb_day + 1) {month++;day = 1;}
					case 3:
						if(day == 32) {month++;day = 1;}
					case 4:
						if(day == 31) {month++;day = 1;}
					case 5:
						if(day == 32) {month++;day = 1;}
					case 6:
						if(day == 31) {month++;day = 1;}
					case 7:
						if(day == 32) {month++;day = 1;}
					case 8:
						if(day == 32) {month++;day = 1;}
					case 9:
						if(day == 31) {month++;day = 1;}
					case 10:
						if(day == 32) {month++;day = 1;}
					case 11:
						if(day == 31) {month++;day = 1;}
					case 12:
						if(day == 32) {day = 1;month = 1;year++;}
				}
			}
		}
	}
	
}

//按键检测函数
void read_menu()
{
	btn_menu -> gpio_read(&key_menu,ADDR_BTN_MENU);
	key_menu = key_menu >> 18;
}

void read_back()
{
	btn_back -> gpio_read(&key_back,ADDR_BTN_BACK);
	key_back = key_back >> 19;
}

void read_up()
{

	btn_up   -> gpio_read(&key_up,ADDR_BTN_UP);9;
	key_up = key_up >> 17;

}

void read_down()
{
	btn_down -> gpio_read(&key_down,ADDR_BTN_DOWN);
	key_down = key_down >> 18;
}
// 菜单函数定义
void func_0()
{	int para_continue = 0;
	EMBARC_PRINTF("- [INFO] WELCOME PAGE.");
	u8g_FirstPage(&u8g);
	do {
		u8g_DrawStr(&u8g, 30, 2, "*Welcome*");
		u8g_DrawStr(&u8g, 20, 16, "Please press");
		u8g_DrawStr(&u8g, 0, 32, "[Menu] or [Back]");
		u8g_DrawStr(&u8g, 20, 48, "to Continue.");
	
		u8g_DrawFrame(&u8g, 26,0,80,16);	//frame for hour
			
		} while (u8g_NextPage(&u8g));
	read_menu();
	read_back();
	while(para_continue)
	{
	if((key_menu)||(key_back)) para_continue = 1;
	EMBARC_PRINTF("- [INFO] EXITING WELCOME PAGE.");
	}
}
void func_1() // 光标移动到Manual Control
{
	menu_screen_p1(1);
}
void func_2() // 光标移动到Setting
{
	menu_screen_p1(2);
}
void func_3() // 光标移动到Main Page
{
	menu_screen_p1(3);
}
void func_4() // 光标移动到Set Alarm
{
	menu_screen_p2(0);
}
void func_5() // 光标移动到Set Time&Date
{
	menu_screen_p2(1);
}
void func_6() //光标移动到Set Mode
{
	menu_screen_p2(2);
}
void func_7() //光标移动到Sensor Celi
{
	menu_screen_p2(3);
}
void func_8() // Manual Control执行函数
{	
	motor_enable(1);
	get_back = 1;
	u8g_FirstPage(&u8g);
	do {
    u8g_DrawStr(&u8g, 8, 0, "Manual Control");
	u8g_DrawStr(&u8g, 0, 28, "-[UP] to Open");
	u8g_DrawStr(&u8g, 0, 44, "-[DOWN] to close");
	} while (u8g_NextPage(&u8g)); 
	
	while(get_back)
	{	
	

		
		read_up();
		read_down();
		if(key_up) 
		{
			EMBARC_PRINTF(" - Press BUTTON UP.\n");
			motor_l -> gpio_write(ADDR_MOT_L,ADDR_MOT_L);
			motor_r -> gpio_write(0x0,ADDR_MOT_R);
			
			EMBARC_PRINTF("in1 -> 1 , in2 -> 0\n");
			while(key_up) read_up();
			EMBARC_PRINTF(" - RELEASE BUTTON UP.\n");
			motor_l -> gpio_write(0x0,ADDR_MOT_L);
			motor_r -> gpio_write(ADDR_MOT_R,ADDR_MOT_R);
			
		}
		if(key_down)
		{	
			EMBARC_PRINTF(" - Press BUTTON DOWN.\n");
			motor_l -> gpio_write(0x0,ADDR_MOT_L);
			motor_r -> gpio_write(ADDR_MOT_R,ADDR_MOT_R);
			EMBARC_PRINTF("in1 -> 0 , in2 -> 1\n");
			while(key_down) read_down();
			EMBARC_PRINTF(" - RELEASE BUTTON DOWN.\n");
			motor_l -> gpio_write(0x0,ADDR_MOT_L);
			motor_r -> gpio_write(0x0,ADDR_MOT_R);
		}	
		delay(10);
		read_menu();
		read_back();
		if((key_menu==1)||(key_back==1)) 
		{
			delay(10);
			if((key_menu==1)||(key_back==1)) get_back = 0;
			EMBARC_PRINTF("- [INFO] func M C Exiting\n");

		}
	}
	motor_enable(0);
}
void func_9() // Main Page 执行函数
{
	int pmw;
	int light_weak;
	int light_meduim;
	int light_strong;
	get_back = 1;
	int n = 100;
	switch(para_sensor)
	{
		case 3:
		{
			light_strong = brit_data;
			light_meduim = brit_data - 40;
			light_weak   = brit_data - 60;
			break;
		}
		case 2:
		{
			light_strong = brit_data + 40;
			light_meduim = brit_data;
			light_weak   = brit_data - 20;
			break;
		}
		case 1:
		{
			light_strong = brit_data + 60;
			light_meduim = brit_data + 40;
			light_weak   = brit_data;
			break;
		}
	}
	while(get_back)
	{
		if((year%4==0&&year%100!=0)||(year%400==0)) Feb_day =29;// 闰年判断
		else Feb_day == 28;
		uint8_t *rx_buf = (uint8_t *)rx_data;
		uint8_t *brit_buf = (uint8_t *)brit_data;
			ALS_read(rx_buf, BUF_LEN);
			EMBARC_PRINTF(" - Reading data.\r\n");
			EMBARC_PRINTF(" - Data received from PMODALS.\r\n");
		display_data(rx_buf, BUF_LEN);
		switch(func_mode)
		{
			case 1: // Full-Auto Mode
			{	
				if(brit_data>=light_strong)
				{	
					while(n >= 0)
					{
						motor_enable(1);
					}
				}
				if(hh<=12)
				{	
					clk_hh = 7 ;
					clk_mm = 30-brit_data/10;
					pmw = brit_data;
					alarm_clock(clk_hh,clk_mm,pmw,1);
				}
				else
				{
					clk_hh = 18;
					clk_mm = 30+brit_data/10;
					pmw = brit_data;
					alarm_clock(clk_hh,clk_mm,pmw,0);
				
				}
				break;
			}
			case 2: // Semi-Auto Mode
			{
					pmw = brit_data;
					alarm_clock(clk_hh,clk_mm,pmw,1);
					break;
			}
			default:{}
		}
		
		read_menu();
		read_back();
		if((key_menu==1)||(key_back==1)) 
		{
			delay(10);
			if((key_menu==1)||(key_back==1)) get_back = 0;
			EMBARC_PRINTF("- [INFO] func Main Page Exiting\n");
		}
		delay(500);
	}
	EMBARC_PRINTF("- [INFO] Return to the Menu\n");
}
void func_10() // Set Alarm执行函数
{	
	int stage = 1;
	int pre_stage = 1;
		ss = 0;

	int t_hh = clk_hh;
	int t_mm = clk_mm;
	int t_ss = ss;
	
	char temp_h[10] = " ";
	char temp_min[10] = " ";
	char temp_s[10] = " ";

	sprintf(temp_h,"%02d",hh);
	sprintf(temp_min,"%02d",mm);
	sprintf(temp_s,"%02d",ss);
	
	while(stage == 1) // 设定CLK小时
		{	
			EMBARC_PRINTF("- [INFO] Stage %d , Setting CLK hour.\n",stage);
			while(stage == pre_stage)
			{
				delay(10);
					sprintf(temp_h,"%02d",clk_hh);
					sprintf(temp_min,"%02d",clk_mm);
					sprintf(temp_s,"%02d",ss);
				u8g_FirstPage(&u8g);
				do {
					u8g_DrawStr(&u8g, 20, 0, "*Set Alarm*");
	
					u8g_DrawStr(&u8g, 48, 28,":");
					u8g_DrawStr(&u8g, 72, 28,":");
					u8g_DrawStr(&u8g, 32, 28,temp_h);
					u8g_DrawStr(&u8g, 56, 28,temp_min);
					u8g_DrawStr(&u8g, 80, 28,temp_s);
	
					u8g_DrawFrame(&u8g, 30,27,20,16);	//frame for hour
			
					} while (u8g_NextPage(&u8g));
				
				read_up();
				read_down();
				if(key_up   == 1) 
					{	
						EMBARC_PRINTF(" - t_hh++\n");
						t_hh++;
						while(key_up) read_up();
					}
				if(key_down == 1) 
					{	
						EMBARC_PRINTF(" - t_hh--\n");
						t_hh--;
						while(key_down) read_down();
					}
				if(t_hh == 25) t_hh = 0;
				if(t_hh == -1)  t_hh = 0;	
				clk_hh = t_hh;
				EMBARC_PRINTF("t_hh = %d , clk_hh = %d\n",t_hh,clk_hh);
				//检测是否按下menu键		
				read_menu();
				if(key_menu == 1)
				{
					stage++;
					while(key_menu) read_menu();
				}

			}	
			EMBARC_PRINTF("- [INFO] Stage %d , CLK hour had been setted.\n",stage);
		}
		pre_stage = stage;
		while(stage == 2) // 设定CLK分钟
		{	
			while(stage == pre_stage)
			{	
				EMBARC_PRINTF("- [INFO] Stage %d , Setting CLK minute.\n",stage);
				delay(10);
					sprintf(temp_h,"%02d",clk_hh);
					sprintf(temp_min,"%02d",clk_mm);
					sprintf(temp_s,"%02d",ss);
				u8g_FirstPage(&u8g);
				do {
					u8g_DrawStr(&u8g, 20, 0, "*Set Alarm*");
	
					u8g_DrawStr(&u8g, 48, 28,":");
					u8g_DrawStr(&u8g, 72, 28,":");
					u8g_DrawStr(&u8g, 32, 28,temp_h);
					u8g_DrawStr(&u8g, 56, 28,temp_min);
					u8g_DrawStr(&u8g, 80, 28,temp_s);
	
					u8g_DrawFrame(&u8g, 54,27,20,16);	//frame for min
			
					} while (u8g_NextPage(&u8g));
				
				read_up();
				read_down();
				if(key_up   == 1) 
					{	
						EMBARC_PRINTF(" - t_mm++\n");
						t_mm++;
						while(key_up) read_up();
					}
				if(key_down == 1) 
					{	
						EMBARC_PRINTF(" - t_mm--\n");
						t_mm--;
						while(key_down) read_down();
					}
				if(t_mm == 61) t_mm = 1;
				if(t_mm == -1)  t_mm = 0;	
				clk_mm = t_mm;
				EMBARC_PRINTF("t_mm = %d , clk_mm = %d\n",t_mm,clk_mm);
				//检测是否按下menu键		
				read_menu();
				if(key_menu == 1)
				{
					stage++;
					while(key_menu) read_menu();
				}
			}	
			EMBARC_PRINTF("- [INFO] Stage %d , CLK minute had been setted.\n",stage);
		}
	EMBARC_PRINTF("Alarm Setted.\n");
}
void func_11() // Set T&D执行函数
{	
	timer_stop(TIMER_0);
	EMBARC_PRINTF("- [INFO] Stop timer0.\n");

	ss = 0;
	int stage = 1;
	int pre_stage = 1;

	int t_hh = hh;
	int t_mm = mm;
	int t_ss = ss;
	int t_yy = year;
	int t_m = month;
	int t_dd = day;
	
	char temp_h[10] = " ";
	char temp_min[10] = " ";
	char temp_s[10] = " ";
	char temp_y[10] = " ";
	char temp_mon[10] = " ";
	char temp_d[10] = " ";
	
	sprintf(temp_y,"%4d",year);
	sprintf(temp_mon,"%02d",month);
	sprintf(temp_d,"%02d",day);
	sprintf(temp_h,"%02d",hh);
	sprintf(temp_min,"%02d",mm);
	sprintf(temp_s,"%02d",ss);
	
		delay(10);
		while(stage == 1) // 设定年
		{	
			EMBARC_PRINTF("- [INFO] Stage %d , Setting year.\n",stage);
			while(stage == pre_stage)
			{
				delay(10);
					sprintf(temp_y,"%4d",year);
					sprintf(temp_mon,"%02d",month);
					sprintf(temp_d,"%02d",day);
					sprintf(temp_h,"%02d",hh);
					sprintf(temp_min,"%02d",mm);
					sprintf(temp_s,"%02d",ss);
				u8g_FirstPage(&u8g);
				do {
					u8g_DrawStr(&u8g, 28, 0, "*Set T&D*");
	
					u8g_DrawStr(&u8g, 48, 42,":");
					u8g_DrawStr(&u8g, 72, 42,":");
					u8g_DrawStr(&u8g, 32, 42,temp_h);
					u8g_DrawStr(&u8g, 56, 42,temp_min);
					u8g_DrawStr(&u8g, 80, 42,temp_s);
	
					u8g_DrawStr(&u8g, 28, 25,temp_y);
					u8g_DrawStr(&u8g, 68, 25,temp_mon);
					u8g_DrawStr(&u8g, 92, 25,temp_d);
					u8g_DrawStr(&u8g, 84, 25,"-");
					u8g_DrawStr(&u8g, 60, 25,"-");
					u8g_DrawFrame(&u8g, 26,24,36,16);	//frame for year
			
					} while (u8g_NextPage(&u8g));
				
				read_up();
				read_down();
				if(key_up   == 1) 
					{	
						EMBARC_PRINTF("year++\n");
						t_yy++;year = t_yy;
						while(key_up) read_up();
						EMBARC_PRINTF(" - t_yy = %d , year = %d\n",t_yy,year);
					}
				if(key_down == 1) 
					{	
						EMBARC_PRINTF("year--\n");
						t_yy--;year = t_yy;
						while(key_down) read_down();
						EMBARC_PRINTF(" - t_yy = %d , year = %d\n",t_yy,year);
					}

				//检测是否按下menu键		
				read_menu();
				if(key_menu == 1)
				{
					stage++;
					while(key_menu) read_menu();
				}

			}	
			EMBARC_PRINTF("- [INFO] Stage %d , year had been setted.\n",stage);
		}
		pre_stage = stage;
		while(stage == 2) // 设定月
		{	
			EMBARC_PRINTF("- [INFO] Stage %d , Setting month.\n",stage);
			while(stage == pre_stage)
			{
				delay(10);
					sprintf(temp_y,"%4d",year);
					sprintf(temp_mon,"%02d",month);
					sprintf(temp_d,"%02d",day);
					sprintf(temp_h,"%02d",hh);
					sprintf(temp_min,"%02d",mm);
					sprintf(temp_s,"%02d",ss);
				u8g_FirstPage(&u8g);
				do {
					u8g_DrawStr(&u8g, 28, 0, "*Set T&D*");
	
					u8g_DrawStr(&u8g, 48, 42,":");
					u8g_DrawStr(&u8g, 72, 42,":");
					u8g_DrawStr(&u8g, 32, 42,temp_h);
					u8g_DrawStr(&u8g, 56, 42,temp_min);
					u8g_DrawStr(&u8g, 80, 42,temp_s);
	
					u8g_DrawStr(&u8g, 28, 25,temp_y);
					u8g_DrawStr(&u8g, 68, 25,temp_mon);
					u8g_DrawStr(&u8g, 92, 25,temp_d);
					u8g_DrawStr(&u8g, 84, 25,"-");
					u8g_DrawStr(&u8g, 60, 25,"-");
					u8g_DrawFrame(&u8g, 66,24,20,16);	//frame for month
			
					} while (u8g_NextPage(&u8g));
				
				read_up();
				read_down();
				if(key_up   == 1) 
					{	
						EMBARC_PRINTF(" - month++\n");
						t_m++;
						while(key_up) read_up();
						if(t_m == 13) t_m = 1;
						if(t_m == 0)  t_m = 1;
						month = t_m;
						EMBARC_PRINTF(" - t_m = %d , month = %d\n",t_m,month);
					}
				if(key_down == 1) 
					{	
						EMBARC_PRINTF(" - month--\n");
						t_m--;
						while(key_down) read_down();
						if(t_m == 13) t_m = 1;
						if(t_m == 0)  t_m = 1;
						month = t_m;
						EMBARC_PRINTF(" - t_m = %d , month = %d\n",t_m,month);
					}
				//检测是否按下menu键		
				read_menu();
				if(key_menu == 1)
				{
					stage++;
					while(key_menu) read_menu();
				}

			}	
			EMBARC_PRINTF("- [INFO] Stage %d , month had been setted.\n",stage);
		}
		pre_stage = stage;
		while(stage == 3) // 设定日
		{	
			EMBARC_PRINTF(" - Month = %d.\n",month);
			EMBARC_PRINTF("- [INFO] Stage %d , Setting day.\n",stage);
			while(stage == pre_stage)
			{
				delay(10);
					sprintf(temp_y,"%4d",year);
					sprintf(temp_mon,"%02d",month);
					sprintf(temp_d,"%02d",day);
					sprintf(temp_h,"%02d",hh);
					sprintf(temp_min,"%02d",mm);
					sprintf(temp_s,"%02d",ss);
				u8g_FirstPage(&u8g);
				do {
					u8g_DrawStr(&u8g, 28, 0, "*Set T&D*");
	
					u8g_DrawStr(&u8g, 48, 42,":");
					u8g_DrawStr(&u8g, 72, 42,":");
					u8g_DrawStr(&u8g, 32, 42,temp_h);
					u8g_DrawStr(&u8g, 56, 42,temp_min);
					u8g_DrawStr(&u8g, 80, 42,temp_s);
	
					u8g_DrawStr(&u8g, 28, 25,temp_y);
					u8g_DrawStr(&u8g, 68, 25,temp_mon);
					u8g_DrawStr(&u8g, 92, 25,temp_d);
					u8g_DrawStr(&u8g, 84, 25,"-");
					u8g_DrawStr(&u8g, 60, 25,"-");
					u8g_DrawFrame(&u8g, 90,24,20,16);	//frame for day
			
					} while (u8g_NextPage(&u8g));
				
				read_up();
				read_down();
				if(key_up   == 1) 
					{	
						EMBARC_PRINTF(" - day++\n");
						t_dd++;
						EMBARC_PRINTF(" - t_dd = %d\n",t_dd);
						while(key_up) read_up();
					}
				if(key_down == 1) 
					{	
						EMBARC_PRINTF(" - day--\n");
						t_dd--;
						EMBARC_PRINTF(" - t_dd = %d\n",t_dd);
						while(key_down) read_down();
					}
				switch(month)
				{
					case 1:
					{
						if((t_dd > 31)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF("- [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 2:
					{
						if((t_dd > 30)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF("- [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 3:
					{
						if((t_dd > 31)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 4:
					{
						if((t_dd > 30)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 5:
					{
						if((t_dd > 31)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 6:
					{
						if((t_dd > 30)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 7:
					{
						if((t_dd > 31)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 8:
					{
						if((t_dd > 31)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 9:
					{
						if((t_dd > 30)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 10:
					{
						if((t_dd > 31)||(t_dd < 1))  
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
					case 11:
					{
						if((t_dd > 30)||(t_dd < 1)) 
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
						
					case 12:
					{
						if((t_dd > 31)||(t_dd < 1)) 
						{
							t_dd= 1;
							EMBARC_PRINTF(" - [INFO] t_dd set to 1");
							break;
						}
						day = t_dd;
					}
						
					default:{}
				}	
				//检测是否按下menu键		
				read_menu();
				if(key_menu == 1)
				{
					stage++;
					while(key_menu) read_menu();
				}
			}	
			EMBARC_PRINTF(" - [INFO] Stage %d , day had been setted.\n",stage);
		}
		pre_stage = stage;
		while(stage == 4) // 设定小时
		{	
			EMBARC_PRINTF(" - [INFO] Stage %d , Setting hour.\n",stage);
			while(stage == pre_stage)
			{
				delay(10);
					sprintf(temp_y,"%4d",year);
					sprintf(temp_mon,"%02d",month);
					sprintf(temp_d,"%02d",day);
					sprintf(temp_h,"%02d",hh);
					sprintf(temp_min,"%02d",mm);
					sprintf(temp_s,"%02d",ss);
				u8g_FirstPage(&u8g);
				do {
					u8g_DrawStr(&u8g, 28, 0, "*Set T&D*");
	
					u8g_DrawStr(&u8g, 48, 42,":");
					u8g_DrawStr(&u8g, 72, 42,":");
					u8g_DrawStr(&u8g, 32, 42,temp_h);
					u8g_DrawStr(&u8g, 56, 42,temp_min);
					u8g_DrawStr(&u8g, 80, 42,temp_s);
	
					u8g_DrawStr(&u8g, 28, 25,temp_y);
					u8g_DrawStr(&u8g, 68, 25,temp_mon);
					u8g_DrawStr(&u8g, 92, 25,temp_d);
					u8g_DrawStr(&u8g, 84, 25,"-");
					u8g_DrawStr(&u8g, 60, 25,"-");
					u8g_DrawFrame(&u8g,30,41,20,16);	//frame for hour
			
					} while (u8g_NextPage(&u8g));
				
				read_up();
				read_down();
				if(key_up   == 1) 
					{	
						EMBARC_PRINTF(" - [INFO] t_hh++\n");
						t_hh++;
						if(t_hh == 25) t_hh = 0;
						if(t_hh == -1)  t_hh = 0;	
						hh = t_hh;
						EMBARC_PRINTF(" - t_hh = %d , hour = %d\n",t_hh,hh);
						while(key_up) read_up();
					}
				if(key_down == 1) 
					{	
						EMBARC_PRINTF(" - [INFO] t_hh--\n");
						t_hh--;
						if(t_hh == 25) t_hh = 0;
						if(t_hh == -1)  t_hh = 0;	
						hh = t_hh;
						EMBARC_PRINTF(" - t_hh = %d , hour = %d\n",t_hh,hh);
						while(key_down) read_down();
					}
				//检测是否按下menu键		
				read_menu();
				if(key_menu == 1)
				{
					stage++;
					while(key_menu) read_menu();
				}

			}	
			EMBARC_PRINTF(" - [INFO] Stage %d , hour had been setted.\n",stage);
		}
		pre_stage = stage;
		while(stage == 5) // 设定分钟
		{	
			EMBARC_PRINTF(" - [INFO] Stage %d , Setting minute.\n",stage);
			while(stage == pre_stage)
			{	
				delay(10);
					sprintf(temp_y,"%4d",year);
					sprintf(temp_mon,"%02d",month);
					sprintf(temp_d,"%02d",day);
					sprintf(temp_h,"%02d",hh);
					sprintf(temp_min,"%02d",mm);
					sprintf(temp_s,"%02d",ss);
				u8g_FirstPage(&u8g);
				do {
					u8g_DrawStr(&u8g, 28, 0, "*Set T&D*");
	
					u8g_DrawStr(&u8g, 48, 42,":");
					u8g_DrawStr(&u8g, 72, 42,":");
					u8g_DrawStr(&u8g, 32, 42,temp_h);
					u8g_DrawStr(&u8g, 56, 42,temp_min);
					u8g_DrawStr(&u8g, 80, 42,temp_s);
	
					u8g_DrawStr(&u8g, 28, 25,temp_y);
					u8g_DrawStr(&u8g, 68, 25,temp_mon);
					u8g_DrawStr(&u8g, 92, 25,temp_d);
					u8g_DrawStr(&u8g, 84, 25,"-");
					u8g_DrawStr(&u8g, 60, 25,"-");
					u8g_DrawFrame(&u8g, 54,41,20,16);	//frame for min
			
					} while (u8g_NextPage(&u8g));
				
				read_up();
				read_down();
				if(key_up   == 1) 
					{	
						EMBARC_PRINTF(" - [INFO] mm++\n");
						t_mm++;
						if(t_mm == 61) t_mm = 1;	
						mm = t_mm;
						EMBARC_PRINTF(" - t_mm = %d , min = %d\n",t_mm,mm);
						while(key_up) read_up();
					}
				if(key_down == 1) 
					{	
						EMBARC_PRINTF("mm--\n");
						t_mm--;
						if(t_mm == 0)  t_mm = 1;	
						mm = t_mm;
						EMBARC_PRINTF(" - t_mm = %d , min = %d\n",t_mm,mm);
						while(key_down) read_down();
					}
				//检测是否按下menu键		
				read_menu();
				if(key_menu == 1)
				{
					stage++;
					while(key_menu) read_menu();
				}
			}	
			EMBARC_PRINTF(" - [INFO] Stage %d , minute had been setted.\n",stage);
		}
				

				

				delay(10);
				timer_start(TIMER_0, TIMER_CTRL_IE, BOARD_CPU_CLOCK);
				EMBARC_PRINTF(" - [INFO] T&D seting completed . Timer0 start.\n");
	
}

void func_12() // Set Mode执行函数 func_mode
{	
	char temp_mode[10] = " ";
	int stage = 1;
	sprintf(temp_mode,"%d",func_mode);
	while(stage)
	{	
		read_up();
		read_down();
		if(key_up) 
		{	
			func_mode = 2; // Semi-Auto

			while(key_up) read_up();
		}
		if(key_down) 
		{
			func_mode = 1; // Full-Auto
			
			EMBARC_PRINTF("func_mode = %d\n",func_mode);
			while(key_down) read_down();
		}
		u8g_FirstPage(&u8g);
		do {
			u8g_DrawStr(&u8g, 24, 0, "*Set Mode*");
			u8g_DrawStr(&u8g, 2, 16, "1.Full-Auto");
			u8g_DrawStr(&u8g, 2, 32, "2.Semi-Auto");
			u8g_DrawStr(&u8g, 2, 48, "Select:");
			u8g_DrawFrame(&u8g,58,47,10,16);

			sprintf(temp_mode,"%d",func_mode);

			EMBARC_PRINTF("temp_mode = %s\n",temp_mode);
;
			u8g_DrawStr(&u8g, 59, 48, temp_mode);

		} while (u8g_NextPage(&u8g)); 
		read_menu();
		if(key_menu)
		{	
			delay(10);
			if(key_menu)
			{	
				stage = 0;
				while(key_menu) read_menu();
			}
		}
	}
	EMBARC_PRINTF("func_12 Mode Setting EXIT");
	delay(10);
}
void func_13() // Sensor Celibration执行函数
{	
	int stage = 1;
	int num = para_sensor;
	
	while(stage)
	{	
		read_up();
		read_down();
		if(key_up) 
		{
			num++;
			while(key_up) read_up();
		}
		if(key_down) 
		{
			num--;
			while(key_down) read_down();
		}	
		if((num == 0)||(num == 4))  num = 1;
		
		switch(num)
		{
			case 3:
			{
				u8g_FirstPage(&u8g);
				do {
				u8g_DrawStr(&u8g, 16, 42, "<- Strong ->");
				} while (u8g_NextPage(&u8g)); 
				para_sensor = num;
				EMBARC_PRINTF("case 1 num = %d\n",num);
				break;
			}
			
			case 2:
			{
				u8g_FirstPage(&u8g);
				do {
				u8g_DrawStr(&u8g, 16, 42, "<- Medium ->");

				} while (u8g_NextPage(&u8g));
				para_sensor = num;
				EMBARC_PRINTF("case 2 num = %d\n",num);
				break;
			}
			case 1:
			{
				u8g_FirstPage(&u8g);
				do {
				u8g_DrawStr(&u8g, 16, 42, "<-  Weak  ->");
				} while (u8g_NextPage(&u8g));
				para_sensor = num;
				EMBARC_PRINTF("case 3 num = %d\n",num);
				break;
			}
			default:{}
		}
		read_menu();
		if(key_menu)
		{	
			delay(10);
			if(key_menu)
			{	
				stage = 0;
				while(key_menu) read_menu();
			}
		}
	}
}