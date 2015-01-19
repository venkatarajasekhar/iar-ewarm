//------------------------------------------------------------------------------
//名称：低电压采集器-红外抄表版本1
//人员：颜培涛
//时间：2013.11.15
//说明：
//当前功耗:采集时10-15mA持续7秒，待机状态稳定时130-150uA.按平均值计算，10分采集间隔时间时，2000mA电池可工作280天左右。
/*
 电源开关：
 采集器具有一个可插拔红外收发头。插入采集器主控盒后，采集器上电工作。拔掉则（完全掉电）关机，可长期仓库放置。
 采集器有两种工作状态：
 插上USB数据线，采集器仅工作在通信状态，可以进行设置和读取数据操作。拔掉USB线则进入采集状态。
 测试按键：
 为了判断采集器是否能够正常采集，安装后应当按一下测试键。
 状态指示灯：
 通信状态时，指示灯常亮，通信时闪烁。采集状态时，常灭。按下测试按键时先亮，如果采集测试成功则1秒间隔闪烁3秒后熄灭，如果采集测试失败，则不闪烁直接熄灭。
 电池更换：
 采集器内部单节电池可维持工作6个月，使用时间到达6个月应当更换电池，注意更换电池时不要连接USB线（电池更换时间不会被记录），拔掉电池的小白插头，等约3秒后（让系统充分掉电），插入新电池即可。
 通信接口：
 USB通信口，可以进行设置，读数据等通信操作，通信速率64字节每毫秒，遵循USB2.0协议，设备描述为HID类设备，即人体工程学设备，免驱动，符合客户不能插U盘的需求。
 具备一个TTL电平，USART通信口，仅用于公司内部测试维护用途。可使用专用转接线连接PC机形成串口，用于命令收发、测试、数据监控功能。
 实时时钟：
 采用带温度补偿时钟芯片，不会因外界温度变化引起较大的时间偏差。由纽扣电池供电，通常不需要更换。
 通信规约有两种：
 1997年645规约与2007年645规约，以下简称97规约，07规约。
 先发送07规约，有回复时，以后只使用07规约(2400,1200两种波特率)。07规约无回复时，发送97规约（两种波特率），97规约有回复则以后只使用97规约。
 97规约无回复则07规约，97规约再重试一次。依然没有回复则本周期结束，等待下一周期继续以上步骤。
 采集数据：
 采集不成功的数据项内容保留为0XFF.注意数据处理。
 数据帧格式，如下。（采集数据内容遵循07规约中的数据格式，采集器内部完成97规约转07规约数据格式）
 【0xa5】【表地址，低-高顺序，共6字节】【采集时间，年月日星期时分秒顺序，共7字节】【电流,ABC】【电压,ABC】【功率因数，总ABC】【有功功率，总ABC】【无功功率，总ABC】
 0xa5,数据头标识。
 表地址，可以标示出数据对应的表号，如果换过表，会变成相应的表号，保证数据的可靠。
 采集时间，标示数据采集的实时时间。
 采集数据，(X表示半个字节，下面数据，2或3个整字节)
 A电流       xxx.xxx
 B电流       xxx.xxx
 C电流       xxx.xxx
 A电压       XXX.X
 A电压       XXX.X
 A电压       XXX.X
 总功率因数  X.XXX
 A功率因数   X.XXX
 B功率因数   X.XXX
 C功率因数   X.XXX
 总有功功率  XX.XXXX
 A有功功率   XX.XXXX
 B有功功率   XX.XXXX
 C有功功率   XX.XXXX
 总无功功率  XX.XXXX
 A无功功率   XX.XXXX
 B无功功率   XX.XXXX
 C无功功率   XX.XXXX
 注意点：
 只要采集器重新上电或复位都会导致数据存储器开辟一个新块进行存储。读数据时如果发现连续的0XFF块是正常的。
 电池更换时间，是系统完全掉电后，重新上电（USB插上时，会判断为不记录此时间），则记录此时间为电池更换时间。因为内部有电容存在，如果更换电池十分迅速,如1秒，则此时间可能不会被记录。
 纽扣电池是时钟保持用途，如果读取的时间偏差极大，则应考虑更换纽扣电池。

 */
//设置时间： 0x5a 0x02 0x08 【秒分时期日月年DCD码】 0xa5   
//13年11月18日周1，10点05分00秒：   5A 02 08 00 05 10 01 18 11 13 A5 
//回复：“SETOK ”
//读取时间： 0x5a 0x02 0x09 0xa5   
// 5A 02 09 A5
//回复：00x5a 0x02 0x09 【秒分时期日月年DCD码】 0xa5
//读取全部历史数据： 0x5a 0x02 0x0b 0xa5   
// 5A 02 0B A5
//回复：前面描述的64字节结构存储数据,如果历史数据已经读完，返回“NULL”。如果想重新读一遍所有历史数据，应先拔掉USB数据线，然后重新插上，发送读取命令即可。
//清空数据： 0x5a 0x02 0x0d 0xa5   
// 5A 02 0D A5
//回复：“SETOK ” //根据已经存储的数据多少，0-2分钟。
//设置采样间隔： 0x5a 0x02 0x10 【1-60分】 0xa5   
//设置10分间隔  5A 02 10 10 A5
//回复：“SETOK ”
//读采样间隔： 0x5a 0x02 0x11 0xa5   
// 5A 02 11 A5
//回复：0x5a 0x02 0x11 【0-60分】 0xa5
//读电池更换时间： 0x5a 0x02 0x12 0xa5   
// 5A 02 12 A5
//回复：0x5a 0x02 0x12 【秒分时星期日月年】 0xa5
//读取存储数据传输次数 0X5A 0X02 0X13 0XA5
// 5A 02 13 A5
//回复：0x5a 0x02 0x13 【cnt0-cnt3,4字节，低位在前】 0xa5
//------------------------------------------------------------------------------
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f10x_adc.h"
#include "platform_config.h"
#include "test.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern vu16 CCR1_Val;
extern u8 In_Buffer[256];
extern u8 Out_Buffer[256];
u8 uart1_data[RXD_SUM];  //串口处理缓冲
u8 uart_rxcont_sum;      //串口读到的数据长度
u8 read_data[256];
u8 read_data_flag = 0;
u16 read_data_cont = 0;
extern vu32 delay_time;
//------------------------------------------------------------------------------
extern vu32 t2_cont;       //通信超时时间。
extern vu8 LED_CTR;       //LED灯状态
extern vu32 led_hold_ms;   //LED灯状态持续时间
extern vu8 scan_test_en; //采集测试允许
extern vu8 key_test_flag; //测试按键被按下过标志。
extern u32 read_page_cont;

u8 top_time[20];          //【0xa5】【年】【月】【日】【星期】【时】【分】【秒】
u16 time_scont[2];        //折合秒数
u8 time_scan_flag = 1;     //时间到了，采集了标志。
u8 top_addr[20] =
{ 6, 0x12, 0x34, 0x56, 0x78, 0x90, 0xab }; //【6】【表地址】
u8 save_data_buf[256] =
{ 0xa5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //FLASH写缓冲 【写标志】【表号】【时间】【数据】
u8 save_data_cont = 0;     //FLASH写计数
u8 led_cont = 0;           //通过唤醒秒计数
u16 read_s_cont = 0;        //为0时表示，应该读时间了
u8 usb_flag = 0;           //1时表示，USB插过。
u8 old_time = 61;          //上一次读的分。
volatile uint8_t scan_en = 0;         //采集允许
u8 scan_flag = 0;          //扫描采集结果标示
u8 biao_addr[20] =
{ 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa }; //表默认地址
//------------------------------------------------------------------------------
u8 set_updata_time = 5;  //采集周期，单位分，范围1-60分。不能为0。
u8 time_hex_10 = 0;
u8 scan_var = 0;    //1 [1200 07] 2 [1200 97]  3 [2400 07] 4 [2400 97] 其他设置认为无效。

//处理器BAT与VCC连接，BAT(VCC)掉电则数据消失------------------------------------
u8 batt_time[7];       //年月日周时分秒，电池上电时间。

//------------------------------------------------------------------------------
void
Delay(vu32 nCount);
u8
scan_vaw(void);  //采集
u8
scan_test(void); //采集测试
void
USB_com(void);   //USB通信
void
uart_com(void);    //UART通信
void adc_refresh(void);

//------------------------------------------------------------------------------
// FE   FE   FE   FE   68   AA   AA   AA   AA   AA   AA   68   11   04   33   34   34   35  B1   16

u8 com_add_cmd07[40] =
{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x68,
		0x11, 0x04, 0x33, 0x34, 0x33, 0x33, 0xae, 0x16 }; //正向有功总电能
/*
 { 20, 0xfe, 0xfe, 0xfe, 0xfe,
 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
 0x68, 0x11, 0x04, 0x33, 0x34, 0x34, 0x35, 0xb1, 0x16 }; //A相电压
 */

/*
 { 20, 0xfe, 0xfe, 0xfe, 0xfe,
 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
 0x68, 0x13, 0x00, 0xdf, 0x16};
 */
u8 com_add_cmd97[40] =
/*
 { 18, 0xFE, 0xFE, 0xFE, 0xFE,
 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0x68, 0x01, 0x02, 0x44, 0xE9, 0xFC, 0x16 };
 */

{ 18, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x68,
		0x01, 0x02, 0x43, 0xc3, 0xd5, 0x16 }; //正向有功总电能

u8 com_cmd07[18][25] =
{

{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x68,
		0x11, 0x04, 0x33, 0x34, 0x35, 0x35, 0xb2, 0x16 },    //A
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x35, 0x35, 0x35, 0xb3, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x36, 0x35, 0x35, 0xb4, 0x16 },

		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x34, 0x34, 0x35, 0xb1, 0x16 },    //V
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x35, 0x34, 0x35, 0xb2, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x36, 0x34, 0x35, 0xb3, 0x16 },

		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x33, 0x39, 0x35, 0xb5, 0x16 },    //Q
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x34, 0x39, 0x35, 0xb6, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x35, 0x39, 0x35, 0xb7, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x36, 0x39, 0x35, 0xb8, 0x16 },

		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x33, 0x36, 0x35, 0xb2, 0x16 },    //KW
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x34, 0x36, 0x35, 0xb3, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x35, 0x36, 0x35, 0xb4, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x36, 0x36, 0x35, 0xb5, 0x16 },

		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x33, 0x37, 0x35, 0xb3, 0x16 },   //KVAR
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x34, 0x37, 0x35, 0xb4, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x35, 0x37, 0x35, 0xb5, 0x16 },
		{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
				0x68, 0x11, 0x04, 0x33, 0x36, 0x37, 0x35, 0xb6, 0x16 } };

u8 com_cmd97[18][40] =
{

{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x54, 0xE9, 0x0C, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x55, 0xE9, 0x0D, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x56, 0xE9, 0x0E, 0x16 },

{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x44, 0xE9, 0xFC, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x45, 0xE9, 0xFD, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x46, 0xE9, 0xFE, 0x16 },

{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x83, 0xE9, 0x3B, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x84, 0xE9, 0x3C, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x85, 0xE9, 0x3D, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x86, 0xE9, 0x3E, 0x16 },

{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x63, 0xE9, 0x1B, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x64, 0xE9, 0x1C, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x65, 0xE9, 0x1D, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x66, 0xE9, 0x1E, 0x16 },

{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x73, 0xE9, 0x2B, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x74, 0xE9, 0x2C, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x75, 0xE9, 0x2D, 0x16 },
{ 18, 0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x68,
		0x01, 0x02, 0x76, 0xE9, 0x2E, 0x16 } };

/*******************************************************************************
 * Function Name  : main.
 * Description    : main routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/

ITStatus it_stat;

#include "seg.h"
#include "adc_voltage.h"

typedef enum
{
	ustandby, usuccess, uerror
} usb_stat;

usb_stat ustat = ustandby;
uint16_t ustat_cd;

volatile seg_type segment;
volatile uint8_t adc_en;
volatile uint8_t wakeup_flag;
volatile uint32_t shutdown_cd = 3000;
volatile uint8_t shutdown_en;
volatile uint32_t powerup_cd = 3000;
volatile uint8_t powerup_en;
const uint8_t ADC_TIMES = 10;
const uint16_t USTAT_KEEP = 0xFFFF;
uint16_t adc_tmp;
uint16_t adc_value;
uint16_t adc_sum;

seg_config seg_cfg;

#define USB_EN() do { GPIO_SetBits(GPIOC, GPIO_Pin_10); } while (0)
#define SHUTDOWN() do { GPIO_SetBits(GPIOC, GPIO_Pin_12); } while (0)

int main(void)
{
	Set_System();             //允许SHE,8MHZ不PLL.
	GPIO_Configuration();     //配置LED,KEY,USB,USB电源管脚。
	NVIC_Config(0);
	USB_Cable_Config(DISABLE);     //令USB无效(IO控制)。

	/**************************************************************************/

	ADC_init();

	seg_cfg.pin_type[0] = GPIOC;
	seg_cfg.pin[0] = GPIO_Pin_1;
	seg_cfg.pin_type[1] = GPIOC;
	seg_cfg.pin[1] = GPIO_Pin_6;
	seg_cfg.pin_type[2] = GPIOC;
	seg_cfg.pin[2] = GPIO_Pin_7;
	seg_cfg.pin_type[3] = GPIOC;
	seg_cfg.pin[3] = GPIO_Pin_9;
	seg_cfg.pin_type[4] = GPIOC;
	seg_cfg.pin[4] = GPIO_Pin_8;
	seg_cfg.pin_type[5] = GPIOC;
	seg_cfg.pin[5] = GPIO_Pin_0;
	seg_cfg.pin_type[6] = GPIOC;
	seg_cfg.pin[6] = GPIO_Pin_3;
	seg_cfg.pin_type[7] = GPIOC;
	seg_cfg.pin[7] = GPIO_Pin_2;
	seg_cfg.pin_type[8] = GPIOB;
	seg_cfg.pin[8] = GPIO_Pin_14;
	seg_cfg.pin_type[9] = GPIOB;
	seg_cfg.pin[9] = GPIO_Pin_15;

	seg_cfg.speed = GPIO_Speed_2MHz;
	seg_cfg.mode = GPIO_Mode_Out_PP;
	seg_cfg.seg_en = SEG_RESET_EN;
	seg_cfg.dig_en = DIG_RESET_EN;
	seg_cfg.rcc = RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB;

	seg_init(&seg_cfg);

	/**************************************************************************/

//	LED_H();                  //LED亮，开机或复位亮一下。
	HWRXD_EN_L();             //关闭红外电源
	uart2_io_enable(0);
	tim1_io_enable(0);
	SPI_FLASH_Init(0);        //SPI低功耗
	spi_io_enable(1);         //必须先给电
	//-------------------------------------------看门狗配置
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_SetReload(0x9c4);          //2s 0x4e2  1s
	IWDG_ReloadCounter();
	WWDG_EN();
	//-----------------------------------------------------
	CCR1_Val = 111; //t2 1ms定时
	TIMER_Init();
	WWDG_WEI();

	/**************************************************************************/
	segment.display = SEG_DISPLAY_ON;
	segment.dig8[0] = ascii2seg_default('0');
	segment.dig8[1] = ascii2seg_default('0');
	/**************************************************************************/

	delay_time = 0;
	while (delay_time < 6000)
	{
		if (delay_time % 20 == 0)
		{
			seg_flush(&segment, &seg_cfg);
			WWDG_WEI();
		}
	}

	/**************************************************************************/
	segment.display = SEG_DISPLAY_OFF;
	seg_flush(&segment, &seg_cfg);
	/**************************************************************************/

	//-----------------------------------------------------
	ds3231_io_init(1);
	//iic_1_sendbyte( 0x0f,0x00 );  //停止32K输出
	//iic_1_sendbyte( 0x07,0x00 );  //禁止闹钟。
	//iic_1_sendbyte( 0x0e,0x00 );  //方波输出1HZ,禁止两个闹钟中断。
	//----------------------------------------------------------------------------
	/*
	 iic_1_sendbyte( 0x06,0x14 );//年
	 iic_1_sendbyte( 0x05,0x03 );//月
	 iic_1_sendbyte( 0x04,0x06 );//日
	 iic_1_sendbyte( 0x03,0x04 );//星期
	 iic_1_sendbyte( 0x02,0x16 );//时z
	 iic_1_sendbyte( 0x01,0x12 );//分
	 iic_1_sendbyte( 0x00,0x30 );//秒
	 */
	/*
	 top_time[1] = iic_1_readbyte( 0x00 ); //秒
	 top_time[2] = iic_1_readbyte( 0x01 );
	 top_time[3] = iic_1_readbyte( 0x02 );
	 top_time[4] = iic_1_readbyte( 0x03 );
	 top_time[5] = iic_1_readbyte( 0x04 );
	 top_time[6] = iic_1_readbyte( 0x05 );
	 top_time[7] = iic_1_readbyte( 0x06 );
	 */
	//----------------------------------------------------------------------------
	//ds3231_io_init(0);
	//用BKP功能保证，只有电池掉电时才清除上电时间记录。
	if (BKP_ReadBackupRegister( BKP_DR1) == 0x5a) //存在有效记录，则读取。
	{
		//batt_time[7]; //年月日周时分秒，电池上电时间。
		batt_time[0] = BKP_ReadBackupRegister( BKP_DR2);   //读备份寄存器
		batt_time[1] = BKP_ReadBackupRegister( BKP_DR3);   //读备份寄存器
		batt_time[2] = BKP_ReadBackupRegister( BKP_DR4);   //读备份寄存器
		batt_time[3] = BKP_ReadBackupRegister( BKP_DR5);   //读备份寄存器
		batt_time[4] = BKP_ReadBackupRegister( BKP_DR6);   //读备份寄存器
		batt_time[5] = BKP_ReadBackupRegister( BKP_DR7);   //读备份寄存器
		batt_time[6] = BKP_ReadBackupRegister( BKP_DR8);   //读备份寄存器
	}
	else if ( USB_TEST_READ() == 0)   //无有效记录，且USB没插上，则记录。
	{
		top_time[1] = iic_1_readbyte(0x00); //秒
		top_time[2] = iic_1_readbyte(0x01);
		top_time[3] = iic_1_readbyte(0x02);
		top_time[4] = iic_1_readbyte(0x03);
		top_time[5] = iic_1_readbyte(0x04);
		top_time[6] = iic_1_readbyte(0x05);
		top_time[7] = iic_1_readbyte(0x06);

		BKP_WriteBackupRegister( BKP_DR1, 0x5a);            //当前显示信箱号写备份寄存器
		BKP_WriteBackupRegister( BKP_DR2, top_time[1]);     //当前显示信箱号写备份寄存器
		BKP_WriteBackupRegister( BKP_DR3, top_time[2]);     //当前显示信箱号写备份寄存器
		BKP_WriteBackupRegister( BKP_DR4, top_time[3]);     //当前显示信箱号写备份寄存器
		BKP_WriteBackupRegister( BKP_DR5, top_time[4]);     //当前显示信箱号写备份寄存器
		BKP_WriteBackupRegister( BKP_DR6, top_time[5]);     //当前显示信箱号写备份寄存器
		BKP_WriteBackupRegister( BKP_DR7, top_time[6]);     //当前显示信箱号写备份寄存器
		BKP_WriteBackupRegister( BKP_DR8, top_time[7]);     //当前显示信箱号写备份寄存器
		batt_time[0] = BKP_ReadBackupRegister( BKP_DR2);   //读备份寄存器
		batt_time[1] = BKP_ReadBackupRegister( BKP_DR3);   //读备份寄存器
		batt_time[2] = BKP_ReadBackupRegister( BKP_DR4);   //读备份寄存器
		batt_time[3] = BKP_ReadBackupRegister( BKP_DR5);   //读备份寄存器
		batt_time[4] = BKP_ReadBackupRegister( BKP_DR6);   //读备份寄存器
		batt_time[5] = BKP_ReadBackupRegister( BKP_DR7);   //读备份寄存器
		batt_time[6] = BKP_ReadBackupRegister( BKP_DR8);   //读备份寄存器
	}
	WWDG_WEI();
	iic_1_sendbyte(0x0f, 0x00);  //停止32K输出
	iic_1_sendbyte(0x07, 0x00);  //禁止闹钟。
	iic_1_sendbyte(0x0e, 0x00);  //方波输出1HZ,禁止两个闹钟中断。
	WWDG_WEI();

	savedata_out();          //第一个字节为0xff则不读取存储数据。
	fand_read_wirte_cont();  //查找写起点【头】读起点【尾】。
	spi_io_enable(0);
	ds3231_io_init(0);

	usb_flag = 0;             //1时表示，USB插过。
	read_s_cont = 30;         //第一次上电30秒后开始第一次读时间。
//	LED_CTR = 2;                //LED指示灯灭。
//	LED_L();
	time_scan_flag = 1;         //采集过了，重新计算唤醒次数。
	delay_time = 0;
	adc_refresh();
	while (1)
	{
		if (shutdown_en == 1)
		{
			segment.display = SEG_DISPLAY_ON;
			segment.dig8[0] = ascii2seg_default('o');
			segment.dig8[1] = ascii2seg_default('F');

			delay_time = 0;
			while (delay_time < 6000)
			{
				if (delay_time % 20 == 0)
				{
					seg_flush(&segment, &seg_cfg);
					WWDG_WEI();
				}
			}

			segment.display = SEG_DISPLAY_OFF;
			seg_flush(&segment, &seg_cfg);

			while (1)
			{
				SHUTDOWN()
				;
				WWDG_WEI();
			}
		}
		else if ( USB_TEST_READ() == 1)    //USB插上，进入设置状态，停止正常采集操作。
		{
			/******************************************************************/
			USB_EN()
			;
			GPIO_SetBits(GPIOB, GPIO_Pin_0);

			if (ustat_cd > 0)
				ustat_cd--;
			else
				ustat = ustandby;

			segment.display = SEG_DISPLAY_ON;
			switch (ustat)
			{
			case ustandby:
				segment.dig8[0] = ascii2seg_default('P');
				segment.dig8[1] = ascii2seg_default('C');
				break;
			case usuccess:
				segment.dig8[0] = ascii2seg_default('E');
				segment.dig8[1] = ascii2seg_default('d');
				break;
			default:
			case uerror:
				segment.dig8[0] = ascii2seg_default('E');
				segment.dig8[1] = ascii2seg_default('r');
				break;
			}

			seg_flush(&segment, &seg_cfg);
			/******************************************************************/

			if (delay_time > 2000) //稳定1秒后
			{
				if (usb_flag == 0)
				{
					usb_flag = 1;         //1时表示，USB插过。

					//LED_CTR = 1;          //LED指示灯亮。
					Set_System();             //允许SHE,8MHZ不PLL.
					GPIO_Configuration();             //配置LED,KEY,USB,USB电源管脚。
					spi_io_enable(1);
					Set_USBClock();     //这里要先把主时钟倍频到72MHZ
					USB_Init();
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
					NVIC_Config(1);

					CCR1_Val = 1000;
					TIMER_Init();
					//UART1_Init();

					SPI_FLASH_Init(1);  //4分频
					ds3231_io_init(1);
					fand_read_wirte_cont();  //查找写起点【头】读起点【尾】。
				}
				if (Out_Buffer[0] == 0x5a)  //USB有接收数据
				{
					//led_hold_ms = 2000;   //快闪1秒
					//LED_CTR = 3;
					USB_com();
				}
				uart_rxcont_sum = get_string(1, uart1_data);
				//if (uart_rxcont_sum > 2) //配置串口有数据
				//{
				//led_hold_ms = 2000;   //快闪1秒
				//LED_CTR = 3;
				//uart_com();         //串口设置，测试。只要插USB时可以用。
				//}
			}
			WWDG_WEI();
		}
		/**********************************************************************/
		else if (adc_en == 1)
		{

			char adc_ascii[2] =
			{ 0, 0 };

			adc_en = 0;
			wakeup_flag = 1;

			adc_refresh();
			//ADC_Cmd(ADC1, ENABLE);
			GPIO_SetBits(GPIOB, GPIO_Pin_0);

			adc2ascii(adc_sum, adc_ascii);

			segment.display = SEG_DISPLAY_ON;
			segment.dig8[0] = ascii2seg_default(adc_ascii[0]);
			segment.dig8[1] = ascii2seg_default(adc_ascii[1]);

			delay_time = 0;
			while (delay_time < 6000)
			{
				if (delay_time % 20 == 0)
				{
					WWDG_WEI();
					seg_flush(&segment, &seg_cfg);
				}
			}

			segment.display = SEG_DISPLAY_OFF;
			seg_flush(&segment, &seg_cfg);

			wakeup_flag = 0;
		}
		else if (scan_test_en == 1)           //采集测试允许
		{
			//segment.display = SEG_DISPLAY_OFF;

			tim1_io_enable(1);
			HWRXD_EN_H();  //打开红外电源
			uart2_io_enable(1);
			scan_flag = scan_test();   //采集测试。
			uart2_io_enable(0);
			HWRXD_EN_L();  //关闭红外电源
			tim1_io_enable(0);

			if (scan_flag > 0)          //测试成功
			{
				savedata_in();         //存储采集参数到内部FLASH;
				char scan_resault[2] =
				{ '0', '0' };
				seg_itoa(scan_flag - 1, scan_resault, 2, 10);
				segment.dig8[0] = ascii2seg_default(scan_resault[0]);
				segment.dig8[1] = ascii2seg_default(scan_resault[1]);
			}
			else //测试失败
			{
				//LED_CTR = 2; //灯灭
				//LED_L();
				//segment.display = SEG_DISPLAY_ON;
				segment.dig8[0] = ascii2seg_default('E');
				segment.dig8[1] = ascii2seg_default('r');
			}

			if (key_test_flag == 1)  //测试按键被按下过标志。
			{
				wakeup_flag = 1;
				key_test_flag = 0;    //测试按键被按下过标志。
				//LED_CTR = 2;          //灯灭
				//led_cont = 6;         //闪烁3秒
				//segment.display = SEG_DISPLAY_ON;

				segment.display = SEG_DISPLAY_ON;

				delay_time = 0;
				while (delay_time < 3000)
				{
					if (delay_time % 20 == 0)
					{
						WWDG_WEI();
						seg_flush(&segment, &seg_cfg);
					}
				}

				segment.display = SEG_DISPLAY_OFF;
			}

			WWDG_WEI();
			wakeup_flag = 0;
			scan_test_en = 0;
		}
		/**********************************************************************/
		else //正常采集工作状态
		{
			delay_time = 0;
			if (usb_flag == 1)       //1时表示，USB插过。重启系统，重新配置。
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
				usb_flag = 0;

				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
				IWDG_SetPrescaler(IWDG_Prescaler_32);
				IWDG_SetReload(0x001);          //2s 0x4e2  1s
				IWDG_ReloadCounter();
				WWDG_EN();
				return_main();   //重启。
				while (1)
					;         //这里狗会起效。
			}
			//===================
			spi_io_enable(0);
			segment.display = SEG_DISPLAY_OFF;
			seg_flush(&segment, &seg_cfg);
			WWDG_WEI();
			PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI); //进入STOP模式
			WWDG_WEI();
			//唤醒后HSI被选为系统时钟。采集时为了温度影响最小，应改为HSE时钟。
			//===================
			/*
			 if (led_cont > 0) //通过唤醒周期完成LED闪烁。
			 {
			 led_cont--;
			 if (led_cont & 0x01)
			 {
			 LED_CTR = 1;
			 LED_H();
			 }
			 else
			 {
			 LED_CTR = 2;
			 LED_L();
			 }
			 }
			 */
			/******************************************************************/
			if (read_s_cont % 30 == 0)
				adc_refresh();
			/******************************************************************/
			if (read_s_cont == 0) //读时间
			{
				ds3231_io_init(1);
				top_time[2] = iic_1_readbyte(0x01); //分
				ds3231_io_init(0);
				time_hex_10 = (top_time[2] >> 4) * 10 + (top_time[2] & 0x0f);

				if (old_time != top_time[2]
						&& (time_hex_10 % set_updata_time) == 0) //基于整时间隔分钟数,这里不能判断秒，有可能错过秒。
				{
					Set_System();           //允许SHE,8MHZ不PLL.
					old_time = top_time[2]; //每次分变化才有可能进来一次。

					scan_en = 1;              //采集允许
					time_scan_flag = 1;       //时间到了，采集了标志。
					spi_io_enable(1);       //写FLash要提前给电。
				}
			}
			else
			{
				read_s_cont--;
			}
			//------------------------------------------------------------------
			if (scan_en == 1)      //采集允许
			{
				tim1_io_enable(1); //38KHZ
				HWRXD_EN_H();  //打开红外电源
				uart2_io_enable(1);
				scan_flag = scan_vaw();  //采集
				uart2_io_enable(0);
				HWRXD_EN_L();  //关闭红外电源
				tim1_io_enable(0);

				if (scan_flag == 0)        //采集成功，哪怕成功一条。
				{
					ds3231_io_init(1);
					top_time[3] = iic_1_readbyte(0x02);
					top_time[4] = iic_1_readbyte(0x03);
					top_time[5] = iic_1_readbyte(0x04);
					top_time[6] = iic_1_readbyte(0x05);
					top_time[7] = iic_1_readbyte(0x06);
					ds3231_io_init(0);
					save_data_cont = 0;       //FLASH写计数
					save_data_buf[save_data_cont++] = 0xa5;  //【写标志】【表号】【时间】【数据】
					save_data_buf[save_data_cont++] = top_addr[1]; //表地址,暂时保留不做。表地址不能只在测试时读取，不测试换位置会出错。
					save_data_buf[save_data_cont++] = top_addr[2];
					save_data_buf[save_data_cont++] = top_addr[3];
					save_data_buf[save_data_cont++] = top_addr[4];
					save_data_buf[save_data_cont++] = top_addr[5];
					save_data_buf[save_data_cont++] = top_addr[6];
					save_data_buf[save_data_cont++] = top_time[7]; //年
					save_data_buf[save_data_cont++] = top_time[6];
					save_data_buf[save_data_cont++] = top_time[5];
					save_data_buf[save_data_cont++] = top_time[4];
					save_data_buf[save_data_cont++] = top_time[3];
					save_data_buf[save_data_cont++] = top_time[2];
					save_data_buf[save_data_cont++] = 0x00; //秒
					//spi_io_enable(1);
					flash_wirte_data(save_data_buf);    //写FLASH数据
					spi_io_enable(0);
				}
				else //采集失败，则采集测试。
				{
					scan_test_en = 1; //采集测试允许
				}
				WWDG_WEI();
				scan_en = 0;
			}
//			it_stat = EXTI_GetITStatus(EXTI_Line0);
			if (time_scan_flag == 1) //采集过了就要计算时间点。
			{
				ds3231_io_init(1);
				top_time[1] = iic_1_readbyte(0x00); //秒
				top_time[2] = iic_1_readbyte(0x01); //分
				ds3231_io_init(0);

				time_scont[0] = (top_time[1] >> 4) * 10 + (top_time[1] & 0x0f);
				time_scont[1] = (top_time[2] >> 4) * 10 + (top_time[2] & 0x0f);
				read_s_cont = set_updata_time
						- (time_scont[1] % set_updata_time);
				read_s_cont = read_s_cont * 60 - time_scont[0]; //计算下一个读时间点，read_s_cont。
				time_scan_flag = 0; //时间到了，采集了标志。
			}
			//------------------------------------------------------------------
		}
	}
}
//------------------------------------------------------------------------------
u8 scan_vaw(void)
{
	u8 i;
	u8 flag = 0;
	u8 len = 0;
	u8 buf[RXD_SUM];
	u8 j, k, m;
	u8 cont1[18] =
	{ 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
	u8 zuo_97to07[18] =
	{ 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3 };
	u8 cont;
	u8 sum;
	u8 sum_len;

//	uint8_t success_count = 0;

	if (scan_var > 6)
	{
		UART2_Init(3);
	}
	else if (scan_var > 4) //5，6偶校验2400
	{
		UART2_Init(2);
	}
	else if (scan_var > 2) //3、4无校验1200
	{
		UART2_Init(1);
	}
	else //1、2偶校验1200
	{
		UART2_Init(0);
	}

	save_data_cont = 14;  //FLASH写计数
	for (i = save_data_cont; i < 64; i++) //前14个字节为【标志1】【地址6】和【时间7】
	{
		save_data_buf[i] = 0xff; //令缓冲区中数据无效。
	}

	for (i = 0; i < 18; i++)
	{
		if (scan_var == 1 || scan_var == 3 || scan_var == 5 || scan_var == 7) //发07命令
		{
			put_string(2, &com_cmd07[i][1], com_cmd07[i][0]); //发07命令 仅第1条命令
		}
		else //发97命令
		{
			put_string(2, &com_cmd97[i][1], com_cmd97[i][0]); //发97命令 仅第1条命令
		}
		t2_cont = 0;
		len = RXD_SUM;
		while (t2_cont <= 2000) //等回复，超时时间2000ms。
		{
			WWDG_WEI();
			len = get_string(2, buf);
			//---------------------------------------------test
			/*
			 scan_var=2;
			 len = 16;
			 buf[0] = 0x68;
			 buf[1] = 0x01;
			 buf[2] = 0x01;
			 buf[3] = 0x01;
			 buf[4] = 0x01;
			 buf[5] = 0x01;
			 buf[6] = 0x01;
			 buf[7] = 0x68;
			 buf[8] = 0x81;
			 buf[9] = 0x05;
			 buf[10] = 0xab;
			 buf[11] = 0xcd;
			 buf[12] = 0x34;
			 buf[13] = 0x35;
			 buf[14] = 0x36;
			 buf[15] = 0x73;
			 buf[16] = 0x16;
			 */
			//---------------------------------------------test
			if (len > 10) //收到数据
			{
				for (j = 6; j < len - 8; j++)
				{
					if (scan_var == 1 || scan_var == 3 || scan_var == 5
							|| scan_var == 7) //发07命令
					{
						if (buf[j] == 0x68 && buf[j + 1] == 0x91
								&& (buf[j + 2] == 7 || buf[j + 2] == 6))
						{
							sum = 0;
							sum_len = buf[j + 2] + 10;
							for (cont = 0; cont < sum_len; cont++)
							{
								sum += buf[j - 7 + cont]; //从第一个0x68开始累加。两个68间隔7
							}
							if (sum == buf[j - 7 + sum_len])  //校验
							{
								if (i == 0)
								{
									top_addr[1] = buf[j - 6];
									top_addr[2] = buf[j - 5];
									top_addr[3] = buf[j - 4];
									top_addr[4] = buf[j - 3];
									top_addr[5] = buf[j - 2];
									top_addr[6] = buf[j - 1];
								}
								m = buf[j + 2] - 4;
								for (k = 0; k < m; k++)
								{
									save_data_buf[save_data_cont + k] = buf[j
											+ 7 + k] - 0x33;  //记录数据07规约
								}
								flag = 1; //成功过
								//success_count++;
								break;//成功，则下一个
							}
						}
					}
					else //97规约解析。
					{
						if (buf[j] == 0x68 && buf[j + 1] == 0x81
								&& (buf[j + 2] == 5 || buf[j + 2] == 4))
						{
							sum = 0;
							sum_len = buf[j + 2] + 10;
							for (cont = 0; cont < sum_len; cont++)
							{
								sum += buf[j - 7 + cont]; //从第一个0x68开始累加。两个68间隔7
							}
							if (sum == buf[j - 7 + sum_len]) //校验
							{
								if (i == 0)
								{
									top_addr[1] = buf[j - 6];
									top_addr[2] = buf[j - 5];
									top_addr[3] = buf[j - 4];
									top_addr[4] = buf[j - 3];
									top_addr[5] = buf[j - 2];
									top_addr[6] = buf[j - 1];
								}
								m = buf[j + 2] - 2;
								for (k = 0; k < m; k++)
								{
									save_data_buf[save_data_cont + k] = buf[j
											+ 5 + k] - 0x33; //记录数据97规约
								}
								//------------------------------------------------------

								if (zuo_97to07[i] == 1) //电流 数据高在后，低在前。
								{
									save_data_buf[save_data_cont + 2] =
											save_data_buf[save_data_cont + 1]
													>> 4; //【2】添加进来。
									save_data_buf[save_data_cont + 1] <<= 4;
									save_data_buf[save_data_cont + 1] +=
											save_data_buf[save_data_cont] >> 4;
									save_data_buf[save_data_cont] <<= 4;
								}
								else if (zuo_97to07[i] == 2) //电压
								{
									save_data_buf[save_data_cont + 1] <<= 4; //【1】的低4位给高4位
									save_data_buf[save_data_cont + 1] +=
											save_data_buf[save_data_cont] >> 4; //【0】的高4位给【1】的低4位。
									save_data_buf[save_data_cont] <<= 4; //【0】的低4位给【0】的高4位
								}
								else if (zuo_97to07[i] == 3) //无功功率
								{
									save_data_buf[save_data_cont + 2] =
											save_data_buf[save_data_cont + 1]; //【2】添加进来。
									save_data_buf[save_data_cont + 1] =
											save_data_buf[save_data_cont];
									save_data_buf[save_data_cont] = 0x00;
								}

								//------------------------------------------------------
								flag = 1; //成功过
								//success_count++;
								break;//成功，则下一个
							}
						}
					}
				}
				break; //接收到无效回复
			}
		}
		if (flag == 0) //如果成功过，则不会进入测试状态。
		//if (success_count == 0)
		{
			return 1; //第一个一定要成功，否则进测试。
		}
		save_data_cont = save_data_cont + cont1[i];
	}
	//return (0 - success_count);
	return 0;
}
//------------------------------------------------------------------------------
u8 scan_test(void)
{
	int ret;

	/******************************************************************************/
	UART2_Init(0); //偶校验1200
	ret = test07();
	if (ret > 0)
		return ret;
	ret = test97();
	if (ret > 0)
		return ret;

	UART2_Init(2);  //偶校验2400
	ret = test07();
	if (ret > 0)
		return ret;
	ret = test97();
	if (ret > 0)
		return ret;

	UART2_Init(1);  //无校验1200
	ret = test07();
	if (ret > 0)
		return ret;
	ret = test97();
	if (ret > 0)
		return ret;

	UART2_Init(3);  //无校验2400
	ret = test07();
	if (ret > 0)
		return ret;
	ret = test97();
	if (ret > 0)
		return ret;

	return 0;
}
//------------------------------------------------------------------------------
void USB_com(void)
{
	u8 i;

	WWDG_WEI();
	for (i = 0; i < 64; i++)
	{
		In_Buffer[i] = 0;
	}
	//设置时间
	if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x08
			&& Out_Buffer[10] == 0xa5)
	{
		iic_1_sendbyte(0x0f, 0x00);  //停止32K输出
		iic_1_sendbyte(0x07, 0x00);  //禁止闹钟。
		iic_1_sendbyte(0x0e, 0x00);  //方波输出1HZ,禁止两个闹钟中断。
		iic_1_sendbyte(0x06, Out_Buffer[3]);  //年
		iic_1_sendbyte(0x05, Out_Buffer[4]);  //月
		iic_1_sendbyte(0x04, Out_Buffer[5]);  //日
		iic_1_sendbyte(0x03, Out_Buffer[6]);  //星期
		iic_1_sendbyte(0x02, Out_Buffer[7]);  //时
		iic_1_sendbyte(0x01, Out_Buffer[8]);  //分
		iic_1_sendbyte(0x00, Out_Buffer[9]);  //秒

		In_Buffer[0] = 'S';
		In_Buffer[1] = 'E';
		In_Buffer[2] = 'T';
		In_Buffer[3] = 'O';
		In_Buffer[4] = 'K';

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;

		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);
	}
	//读取时间
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x09
			&& Out_Buffer[3] == 0xa5)
	{
		In_Buffer[0] = 0x5a;
		In_Buffer[1] = 0x02;
		In_Buffer[2] = 0x09;
		In_Buffer[3] = iic_1_readbyte(0x06); //年
		In_Buffer[4] = iic_1_readbyte(0x05);
		In_Buffer[5] = iic_1_readbyte(0x04);
		In_Buffer[6] = iic_1_readbyte(0x03);
		In_Buffer[7] = iic_1_readbyte(0x02);
		In_Buffer[8] = iic_1_readbyte(0x01);
		In_Buffer[9] = iic_1_readbyte(0x00);
		In_Buffer[10] = 0xa5;
		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;
	}
	//读存储数据
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x0b
			&& Out_Buffer[3] == 0xa5)
	{
		if (read_data_cont == 0)
		{
			read_data_flag = flash_read_data(In_Buffer);     //读数据 应判断状态，是否结束读。
			WWDG_WEI();
		}
		if (read_data_flag == 0)
		{
			UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
			SetEPTxCount(ENDP1, 64);
			SetEPTxValid(ENDP1);
		}
		else
		{
			In_Buffer[0] = 'N';
			In_Buffer[1] = 'U';
			In_Buffer[2] = 'L';
			In_Buffer[3] = 'L';

			UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
			SetEPTxCount(ENDP1, 64);
			SetEPTxValid(ENDP1);
		}

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;
	}
	//擦除存储区
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x0D
			&& Out_Buffer[3] == 0xa5)
	{
		Erase_all_sector();

		In_Buffer[0] = 'S';
		In_Buffer[1] = 'E';
		In_Buffer[2] = 'T';
		In_Buffer[3] = 'O';
		In_Buffer[4] = 'K';

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;

		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);

		t2_cont = 0;
		while (t2_cont <= 2000)
			;
		return_main(); //1秒后重启
	}
	//设置采集数据频率
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x10
			&& Out_Buffer[4] == 0xa5)
	{
		if (Out_Buffer[3] > 0 && Out_Buffer[3] < 61)
		{
			set_updata_time = Out_Buffer[3];
			savedata_in();
			In_Buffer[0] = 'S';
			In_Buffer[1] = 'E';
			In_Buffer[2] = 'T';
			In_Buffer[3] = 'O';
			In_Buffer[4] = 'K';

			ustat = usuccess;
			ustat_cd = USTAT_KEEP;
		}
		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);
	}
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x11
			&& Out_Buffer[3] == 0xa5)
	{
		In_Buffer[0] = 0x5a;
		In_Buffer[1] = 0x02;
		In_Buffer[2] = 0x11;
		In_Buffer[3] = set_updata_time;
		In_Buffer[4] = 0xa5;
		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;
	}
	//读取电池上电时间
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x12
			&& Out_Buffer[3] == 0xa5)
	{
		In_Buffer[0] = 0x5a;
		In_Buffer[1] = 0x02;
		In_Buffer[2] = 0x12;
		In_Buffer[3] = batt_time[0];
		In_Buffer[4] = batt_time[1];
		In_Buffer[5] = batt_time[2];
		In_Buffer[6] = batt_time[3];
		In_Buffer[7] = batt_time[4];
		In_Buffer[8] = batt_time[5];
		In_Buffer[9] = batt_time[6];
		In_Buffer[10] = 0xa5;
		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;
	}
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x13
			&& Out_Buffer[3] == 0xa5)
	{
		In_Buffer[0] = 0x5a;
		In_Buffer[1] = 0x02;
		In_Buffer[2] = 0x13;
		In_Buffer[3] = read_page_cont;
		In_Buffer[4] = read_page_cont >> 8;
		In_Buffer[5] = read_page_cont >> 16;
		In_Buffer[6] = read_page_cont >> 24;
		In_Buffer[7] = 0xa5;
		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;
	}
	/**************************************************************************/
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x14
			&& Out_Buffer[4] == 0xa5)
	{
		FLASH_Unlock();
		while (FLASH_ErasePage(0x0801FC00) != FLASH_COMPLETE)
			;
		while (FLASH_ProgramHalfWord(0x0801FC00, Out_Buffer[3])
				!= FLASH_COMPLETE)
			;

		ADC_Calibration_Value = Out_Buffer[3];

		In_Buffer[0] = 'S';
		In_Buffer[1] = 'E';
		In_Buffer[2] = 'T';
		In_Buffer[3] = 'O';
		In_Buffer[4] = 'K';

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;

		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);
	}
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x15
			&& Out_Buffer[3] == 0xa5)
	{

		In_Buffer[0] = 0x5a;
		In_Buffer[1] = 0x02;
		In_Buffer[2] = 0x15;
		In_Buffer[3] = ADC_Calibration_Value;
		In_Buffer[4] = 0xa5;

		ustat = usuccess;
		ustat_cd = USTAT_KEEP;

		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);
	}
	/**************************************************************************/
	else
	{
		In_Buffer[0] = 'E';
		In_Buffer[1] = 'R';
		In_Buffer[2] = 'R';
		In_Buffer[3] = 'O';
		In_Buffer[4] = 'R';

		ustat = uerror;
		ustat_cd = USTAT_KEEP;

		UserToPMABufferCopy(In_Buffer, GetEPTxAddr(ENDP1), 64);
		SetEPTxCount(ENDP1, 64);
		SetEPTxValid(ENDP1);
	}
	for (i = 0; i < 64; i++)
	{
		Out_Buffer[i] = 0;
	}
}
//------------------------------------------------------------------------------
#if 0
void uart_com(void)
{
	if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 11 && uart1_data[1] == 0x02
			&& uart1_data[2] == 0x08 && uart1_data[10] == 0xa5)
	{
		iic_1_sendbyte(0x0f, 0x00);  //停止32K输出
		iic_1_sendbyte(0x07, 0x00);//禁止闹钟。
		iic_1_sendbyte(0x0e, 0x00);//方波输出1HZ,禁止两个闹钟中断。
		iic_1_sendbyte(0x06, uart1_data[3]);//年
		iic_1_sendbyte(0x05, uart1_data[4]);//
		iic_1_sendbyte(0x04, uart1_data[5]);//
		iic_1_sendbyte(0x03, uart1_data[6]);//星期
		iic_1_sendbyte(0x02, uart1_data[7]);//
		iic_1_sendbyte(0x01, uart1_data[8]);//
		iic_1_sendbyte(0x00, uart1_data[9]);//

		put_string(1, "SETOK", 5);
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 4
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x09
			&& uart1_data[3] == 0xa5)
	{
		uart1_data[0] = 0x5a;
		uart1_data[1] = 0x02;
		uart1_data[2] = 0x09;
		uart1_data[3] = iic_1_readbyte(0x06);
		uart1_data[4] = iic_1_readbyte(0x05);
		uart1_data[5] = iic_1_readbyte(0x04);
		uart1_data[6] = iic_1_readbyte(0x03);
		uart1_data[7] = iic_1_readbyte(0x02);
		uart1_data[8] = iic_1_readbyte(0x01);
		uart1_data[9] = iic_1_readbyte(0x00);  //秒
		uart1_data[10] = 0xa5;

		put_string(1, uart1_data, 11);
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 4
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x0b
			&& uart1_data[3] == 0xa5)
	{
		if (read_data_cont == 0)
		{
			read_data_flag = flash_read_data(read_data);     //读数据 应判断状态，是否结束读。
		}
		if (read_data_flag == 0)
		{
			put_string(1, (read_data), 64);
		}
		else
		{
			put_string(1, "NULL", 4);
		}
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 4
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x0D
			&& uart1_data[3] == 0xa5)
	{
		Erase_all_sector();

		put_string(1, "SETOK", 5);
		t2_cont = 0;
		while (t2_cont <= 2000)
		;
		return_main(); //1秒后重启
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 5
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x10
			&& uart1_data[4] == 0xa5)
	{
		if (uart1_data[3] > 0 && uart1_data[3] < 61)
		set_updata_time = uart1_data[3];
		savedata_in();
		put_string(1, "SETOK", 5);
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 4
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x11
			&& uart1_data[3] == 0xa5)
	{
		uart1_data[0] = 0x5a;
		uart1_data[1] = 0x02;
		uart1_data[2] = 0x11;
		uart1_data[3] = set_updata_time;
		uart1_data[4] = 0xa5;
		put_string(1, uart1_data, 5);
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 4
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x12
			&& uart1_data[3] == 0xa5)
	{
		uart1_data[0] = 0x5a;
		uart1_data[1] = 0x02;
		uart1_data[2] = 0x12;
		uart1_data[3] = batt_time[0];
		uart1_data[4] = batt_time[1];
		uart1_data[5] = batt_time[2];
		uart1_data[6] = batt_time[3];
		uart1_data[7] = batt_time[4];
		uart1_data[8] = batt_time[5];
		uart1_data[9] = batt_time[6];
		uart1_data[10] = 0xa5;
		put_string(1, uart1_data, 11);
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 4
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x13
			&& uart1_data[3] == 0xa5)
	{
		uart1_data[0] = 0x5a;
		uart1_data[1] = 0x02;
		uart1_data[2] = 0x13;
		uart1_data[3] = read_page_cont;
		uart1_data[4] = read_page_cont >> 8;
		uart1_data[5] = read_page_cont >> 16;
		uart1_data[6] = read_page_cont >> 24;
		uart1_data[7] = 0xa5;
		put_string(1, uart1_data, 8);
	}
	else
	{
		put_string(1, "ERROR", 5);
	}
}
#endif
/*******************************************************************************
 * Function Name  : Delay
 * Description    : Inserts a delay time.
 * Input          : nCount: specifies the delay time length.
 * Output         : None
 * Return         : None
 *******************************************************************************/
void Delay(vu32 nCount)
{
	for (; nCount != 0; nCount--)
		;
}

#ifdef  DEBUG
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(u8* file, u32 line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1)
	{
	}
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

void adc_refresh(void)
{
	ADC_Cmd(ADC1, ENABLE);
	GPIO_SetBits(GPIOB, GPIO_Pin_0);

	//Enable ADC1 reset calibration register
	ADC_ResetCalibration(ADC1);
	//Check the end of ADC1 reset calibration register
	while (ADC_GetResetCalibrationStatus(ADC1) != RESET)
		;
	//Start ADC1 calibration
	ADC_StartCalibration(ADC1);
	//Check the end of ADC1 calibration
	while (ADC_GetCalibrationStatus(ADC1) != RESET)
		;

	adc_value = 0;
	for (int i = 0; i < ADC_TIMES;)
	{
		delay_time = 0;
		while (delay_time < 50)
			if (delay_time != 0)
				WWDG_WEI();

		adc_tmp = ADC_get();
		if (adc_tmp > 0xC00)
		{
			adc_value += adc_tmp;
			i++;
		}
	}

	adc_value /= ADC_TIMES;
	adc_value = adc_value * (2873 + ADC_Calibration_Value) / 3000;
	if (adc_sum != 0)
		adc_sum = (adc_sum + adc_value) / 2;
	else
		adc_sum = adc_value;

	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	ADC_Cmd(ADC1, DISABLE);

	if (adc_sum < 0xca8)
	{
		segment.display = SEG_DISPLAY_ON;
		segment.dig8[0] = ascii2seg_default('L');
		segment.dig8[1] = ascii2seg_default('o');

		delay_time = 0;
		while (delay_time < 6000)
		{
			if (delay_time % 20 == 0)
			{
				WWDG_WEI();
				seg_flush(&segment, &seg_cfg);
			}
		}

		segment.display = SEG_DISPLAY_OFF;
		seg_flush(&segment, &seg_cfg);

		while (1)
		{
			SHUTDOWN()
			;
			WWDG_WEI();
		}
	}
}
