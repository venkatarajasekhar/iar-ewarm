//------------------------------------------------------------------------------
//���ƣ��͵�ѹ�ɼ���-���Ⳮ��汾1
//��Ա��������
//ʱ�䣺2013.11.15
//˵����
//��ǰ����:�ɼ�ʱ10-15mA����7�룬����״̬�ȶ�ʱ130-150uA.��ƽ��ֵ���㣬10�ֲɼ����ʱ��ʱ��2000mA��ؿɹ���280�����ҡ�
/*
 ��Դ���أ�
 �ɼ�������һ���ɲ�κ����շ�ͷ������ɼ������غк󣬲ɼ����ϵ繤�����ε�����ȫ���磩�ػ����ɳ��ڲֿ���á�
 �ɼ��������ֹ���״̬��
 ����USB�����ߣ��ɼ�����������ͨ��״̬�����Խ������úͶ�ȡ���ݲ������ε�USB�������ɼ�״̬��
 ���԰�����
 Ϊ���жϲɼ����Ƿ��ܹ������ɼ�����װ��Ӧ����һ�²��Լ���
 ״ָ̬ʾ�ƣ�
 ͨ��״̬ʱ��ָʾ�Ƴ�����ͨ��ʱ��˸���ɼ�״̬ʱ�����𡣰��²��԰���ʱ����������ɼ����Գɹ���1������˸3���Ϩ������ɼ�����ʧ�ܣ�����˸ֱ��Ϩ��
 ��ظ�����
 �ɼ����ڲ����ڵ�ؿ�ά�ֹ���6���£�ʹ��ʱ�䵽��6����Ӧ��������أ�ע��������ʱ��Ҫ����USB�ߣ���ظ���ʱ�䲻�ᱻ��¼�����ε���ص�С�ײ�ͷ����Լ3�����ϵͳ��ֵ��磩�������µ�ؼ��ɡ�
 ͨ�Žӿڣ�
 USBͨ�ſڣ����Խ������ã������ݵ�ͨ�Ų�����ͨ������64�ֽ�ÿ���룬��ѭUSB2.0Э�飬�豸����ΪHID���豸�������幤��ѧ�豸�������������Ͽͻ����ܲ�U�̵�����
 �߱�һ��TTL��ƽ��USARTͨ�ſڣ������ڹ�˾�ڲ�����ά����;����ʹ��ר��ת��������PC���γɴ��ڣ����������շ������ԡ����ݼ�ع��ܡ�
 ʵʱʱ�ӣ�
 ���ô��¶Ȳ���ʱ��оƬ������������¶ȱ仯����ϴ��ʱ��ƫ���Ŧ�۵�ع��磬ͨ������Ҫ������
 ͨ�Ź�Լ�����֣�
 1997��645��Լ��2007��645��Լ�����¼��97��Լ��07��Լ��
 �ȷ���07��Լ���лظ�ʱ���Ժ�ֻʹ��07��Լ(2400,1200���ֲ�����)��07��Լ�޻ظ�ʱ������97��Լ�����ֲ����ʣ���97��Լ�лظ����Ժ�ֻʹ��97��Լ��
 97��Լ�޻ظ���07��Լ��97��Լ������һ�Ρ���Ȼû�лظ������ڽ������ȴ���һ���ڼ������ϲ��衣
 �ɼ����ݣ�
 �ɼ����ɹ������������ݱ���Ϊ0XFF.ע�����ݴ���
 ����֡��ʽ�����¡����ɼ�����������ѭ07��Լ�е����ݸ�ʽ���ɼ����ڲ����97��Լת07��Լ���ݸ�ʽ��
 ��0xa5�������ַ����-��˳�򣬹�6�ֽڡ����ɼ�ʱ�䣬����������ʱ����˳�򣬹�7�ֽڡ�������,ABC������ѹ,ABC����������������ABC�����й����ʣ���ABC�����޹����ʣ���ABC��
 0xa5,����ͷ��ʶ��
 ���ַ�����Ա�ʾ�����ݶ�Ӧ�ı�ţ����������������Ӧ�ı�ţ���֤���ݵĿɿ���
 �ɼ�ʱ�䣬��ʾ���ݲɼ���ʵʱʱ�䡣
 �ɼ����ݣ�(X��ʾ����ֽڣ��������ݣ�2��3�����ֽ�)
 A����       xxx.xxx
 B����       xxx.xxx
 C����       xxx.xxx
 A��ѹ       XXX.X
 A��ѹ       XXX.X
 A��ѹ       XXX.X
 �ܹ�������  X.XXX
 A��������   X.XXX
 B��������   X.XXX
 C��������   X.XXX
 ���й�����  XX.XXXX
 A�й�����   XX.XXXX
 B�й�����   XX.XXXX
 C�й�����   XX.XXXX
 ���޹�����  XX.XXXX
 A�޹�����   XX.XXXX
 B�޹�����   XX.XXXX
 C�޹�����   XX.XXXX
 ע��㣺
 ֻҪ�ɼ��������ϵ��λ���ᵼ�����ݴ洢������һ���¿���д洢��������ʱ�������������0XFF���������ġ�
 ��ظ���ʱ�䣬��ϵͳ��ȫ����������ϵ磨USB����ʱ�����ж�Ϊ����¼��ʱ�䣩�����¼��ʱ��Ϊ��ظ���ʱ�䡣��Ϊ�ڲ��е��ݴ��ڣ�����������ʮ��Ѹ��,��1�룬���ʱ����ܲ��ᱻ��¼��
 Ŧ�۵����ʱ�ӱ�����;�������ȡ��ʱ��ƫ�����Ӧ���Ǹ���Ŧ�۵�ء�

 */
//����ʱ�䣺 0x5a 0x02 0x08 �����ʱ��������DCD�롿 0xa5   
//13��11��18����1��10��05��00�룺   5A 02 08 00 05 10 01 18 11 13 A5 
//�ظ�����SETOK ��
//��ȡʱ�䣺 0x5a 0x02 0x09 0xa5   
// 5A 02 09 A5
//�ظ���00x5a 0x02 0x09 �����ʱ��������DCD�롿 0xa5
//��ȡȫ����ʷ���ݣ� 0x5a 0x02 0x0b 0xa5   
// 5A 02 0B A5
//�ظ���ǰ��������64�ֽڽṹ�洢����,�����ʷ�����Ѿ����꣬���ء�NULL������������¶�һ��������ʷ���ݣ�Ӧ�Ȱε�USB�����ߣ�Ȼ�����²��ϣ����Ͷ�ȡ����ɡ�
//������ݣ� 0x5a 0x02 0x0d 0xa5   
// 5A 02 0D A5
//�ظ�����SETOK �� //�����Ѿ��洢�����ݶ��٣�0-2���ӡ�
//���ò�������� 0x5a 0x02 0x10 ��1-60�֡� 0xa5   
//����10�ּ��  5A 02 10 10 A5
//�ظ�����SETOK ��
//����������� 0x5a 0x02 0x11 0xa5   
// 5A 02 11 A5
//�ظ���0x5a 0x02 0x11 ��0-60�֡� 0xa5
//����ظ���ʱ�䣺 0x5a 0x02 0x12 0xa5   
// 5A 02 12 A5
//�ظ���0x5a 0x02 0x12 �����ʱ���������꡿ 0xa5
//��ȡ�洢���ݴ������ 0X5A 0X02 0X13 0XA5
// 5A 02 13 A5
//�ظ���0x5a 0x02 0x13 ��cnt0-cnt3,4�ֽڣ���λ��ǰ�� 0xa5
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
u8 uart1_data[RXD_SUM];  //���ڴ�����
u8 uart_rxcont_sum;      //���ڶ��������ݳ���
u8 read_data[256];
u8 read_data_flag = 0;
u16 read_data_cont = 0;
extern vu32 delay_time;
//------------------------------------------------------------------------------
extern vu32 t2_cont;       //ͨ�ų�ʱʱ�䡣
extern vu8 LED_CTR;       //LED��״̬
extern vu32 led_hold_ms;   //LED��״̬����ʱ��
extern vu8 scan_test_en; //�ɼ���������
extern vu8 key_test_flag; //���԰��������¹���־��
extern u32 read_page_cont;

u8 top_time[20];          //��0xa5�����꡿���¡����ա������ڡ���ʱ�����֡����롿
u16 time_scont[2];        //�ۺ�����
u8 time_scan_flag = 1;     //ʱ�䵽�ˣ��ɼ��˱�־��
u8 top_addr[20] =
{ 6, 0x12, 0x34, 0x56, 0x78, 0x90, 0xab }; //��6�������ַ��
u8 save_data_buf[256] =
{ 0xa5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //FLASHд���� ��д��־������š���ʱ�䡿�����ݡ�
u8 save_data_cont = 0;     //FLASHд����
u8 led_cont = 0;           //ͨ�����������
u16 read_s_cont = 0;        //Ϊ0ʱ��ʾ��Ӧ�ö�ʱ����
u8 usb_flag = 0;           //1ʱ��ʾ��USB�����
u8 old_time = 61;          //��һ�ζ��ķ֡�
volatile uint8_t scan_en = 0;         //�ɼ�����
u8 scan_flag = 0;          //ɨ��ɼ������ʾ
u8 biao_addr[20] =
{ 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa }; //��Ĭ�ϵ�ַ
//------------------------------------------------------------------------------
u8 set_updata_time = 5;  //�ɼ����ڣ���λ�֣���Χ1-60�֡�����Ϊ0��
u8 time_hex_10 = 0;
u8 scan_var = 0;    //1 [1200 07] 2 [1200 97]  3 [2400 07] 4 [2400 97] ����������Ϊ��Ч��

//������BAT��VCC���ӣ�BAT(VCC)������������ʧ------------------------------------
u8 batt_time[7];       //��������ʱ���룬����ϵ�ʱ�䡣

//------------------------------------------------------------------------------
void
Delay(vu32 nCount);
u8
scan_vaw(void);  //�ɼ�
u8
scan_test(void); //�ɼ�����
void
USB_com(void);   //USBͨ��
void
uart_com(void);    //UARTͨ��
void adc_refresh(void);

//------------------------------------------------------------------------------
// FE   FE   FE   FE   68   AA   AA   AA   AA   AA   AA   68   11   04   33   34   34   35  B1   16

u8 com_add_cmd07[40] =
{ 20, 0xfe, 0xfe, 0xfe, 0xfe, 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x68,
		0x11, 0x04, 0x33, 0x34, 0x33, 0x33, 0xae, 0x16 }; //�����й��ܵ���
/*
 { 20, 0xfe, 0xfe, 0xfe, 0xfe,
 0x68, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
 0x68, 0x11, 0x04, 0x33, 0x34, 0x34, 0x35, 0xb1, 0x16 }; //A���ѹ
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
		0x01, 0x02, 0x43, 0xc3, 0xd5, 0x16 }; //�����й��ܵ���

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
	Set_System();             //����SHE,8MHZ��PLL.
	GPIO_Configuration();     //����LED,KEY,USB,USB��Դ�ܽš�
	NVIC_Config(0);
	USB_Cable_Config(DISABLE);     //��USB��Ч(IO����)��

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

//	LED_H();                  //LED����������λ��һ�¡�
	HWRXD_EN_L();             //�رպ����Դ
	uart2_io_enable(0);
	tim1_io_enable(0);
	SPI_FLASH_Init(0);        //SPI�͹���
	spi_io_enable(1);         //�����ȸ���
	//-------------------------------------------���Ź�����
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_SetReload(0x9c4);          //2s 0x4e2  1s
	IWDG_ReloadCounter();
	WWDG_EN();
	//-----------------------------------------------------
	CCR1_Val = 111; //t2 1ms��ʱ
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
	//iic_1_sendbyte( 0x0f,0x00 );  //ֹͣ32K���
	//iic_1_sendbyte( 0x07,0x00 );  //��ֹ���ӡ�
	//iic_1_sendbyte( 0x0e,0x00 );  //�������1HZ,��ֹ���������жϡ�
	//----------------------------------------------------------------------------
	/*
	 iic_1_sendbyte( 0x06,0x14 );//��
	 iic_1_sendbyte( 0x05,0x03 );//��
	 iic_1_sendbyte( 0x04,0x06 );//��
	 iic_1_sendbyte( 0x03,0x04 );//����
	 iic_1_sendbyte( 0x02,0x16 );//ʱz
	 iic_1_sendbyte( 0x01,0x12 );//��
	 iic_1_sendbyte( 0x00,0x30 );//��
	 */
	/*
	 top_time[1] = iic_1_readbyte( 0x00 ); //��
	 top_time[2] = iic_1_readbyte( 0x01 );
	 top_time[3] = iic_1_readbyte( 0x02 );
	 top_time[4] = iic_1_readbyte( 0x03 );
	 top_time[5] = iic_1_readbyte( 0x04 );
	 top_time[6] = iic_1_readbyte( 0x05 );
	 top_time[7] = iic_1_readbyte( 0x06 );
	 */
	//----------------------------------------------------------------------------
	//ds3231_io_init(0);
	//��BKP���ܱ�֤��ֻ�е�ص���ʱ������ϵ�ʱ���¼��
	if (BKP_ReadBackupRegister( BKP_DR1) == 0x5a) //������Ч��¼�����ȡ��
	{
		//batt_time[7]; //��������ʱ���룬����ϵ�ʱ�䡣
		batt_time[0] = BKP_ReadBackupRegister( BKP_DR2);   //�����ݼĴ���
		batt_time[1] = BKP_ReadBackupRegister( BKP_DR3);   //�����ݼĴ���
		batt_time[2] = BKP_ReadBackupRegister( BKP_DR4);   //�����ݼĴ���
		batt_time[3] = BKP_ReadBackupRegister( BKP_DR5);   //�����ݼĴ���
		batt_time[4] = BKP_ReadBackupRegister( BKP_DR6);   //�����ݼĴ���
		batt_time[5] = BKP_ReadBackupRegister( BKP_DR7);   //�����ݼĴ���
		batt_time[6] = BKP_ReadBackupRegister( BKP_DR8);   //�����ݼĴ���
	}
	else if ( USB_TEST_READ() == 0)   //����Ч��¼����USBû���ϣ����¼��
	{
		top_time[1] = iic_1_readbyte(0x00); //��
		top_time[2] = iic_1_readbyte(0x01);
		top_time[3] = iic_1_readbyte(0x02);
		top_time[4] = iic_1_readbyte(0x03);
		top_time[5] = iic_1_readbyte(0x04);
		top_time[6] = iic_1_readbyte(0x05);
		top_time[7] = iic_1_readbyte(0x06);

		BKP_WriteBackupRegister( BKP_DR1, 0x5a);            //��ǰ��ʾ�����д���ݼĴ���
		BKP_WriteBackupRegister( BKP_DR2, top_time[1]);     //��ǰ��ʾ�����д���ݼĴ���
		BKP_WriteBackupRegister( BKP_DR3, top_time[2]);     //��ǰ��ʾ�����д���ݼĴ���
		BKP_WriteBackupRegister( BKP_DR4, top_time[3]);     //��ǰ��ʾ�����д���ݼĴ���
		BKP_WriteBackupRegister( BKP_DR5, top_time[4]);     //��ǰ��ʾ�����д���ݼĴ���
		BKP_WriteBackupRegister( BKP_DR6, top_time[5]);     //��ǰ��ʾ�����д���ݼĴ���
		BKP_WriteBackupRegister( BKP_DR7, top_time[6]);     //��ǰ��ʾ�����д���ݼĴ���
		BKP_WriteBackupRegister( BKP_DR8, top_time[7]);     //��ǰ��ʾ�����д���ݼĴ���
		batt_time[0] = BKP_ReadBackupRegister( BKP_DR2);   //�����ݼĴ���
		batt_time[1] = BKP_ReadBackupRegister( BKP_DR3);   //�����ݼĴ���
		batt_time[2] = BKP_ReadBackupRegister( BKP_DR4);   //�����ݼĴ���
		batt_time[3] = BKP_ReadBackupRegister( BKP_DR5);   //�����ݼĴ���
		batt_time[4] = BKP_ReadBackupRegister( BKP_DR6);   //�����ݼĴ���
		batt_time[5] = BKP_ReadBackupRegister( BKP_DR7);   //�����ݼĴ���
		batt_time[6] = BKP_ReadBackupRegister( BKP_DR8);   //�����ݼĴ���
	}
	WWDG_WEI();
	iic_1_sendbyte(0x0f, 0x00);  //ֹͣ32K���
	iic_1_sendbyte(0x07, 0x00);  //��ֹ���ӡ�
	iic_1_sendbyte(0x0e, 0x00);  //�������1HZ,��ֹ���������жϡ�
	WWDG_WEI();

	savedata_out();          //��һ���ֽ�Ϊ0xff�򲻶�ȡ�洢���ݡ�
	fand_read_wirte_cont();  //����д��㡾ͷ������㡾β����
	spi_io_enable(0);
	ds3231_io_init(0);

	usb_flag = 0;             //1ʱ��ʾ��USB�����
	read_s_cont = 30;         //��һ���ϵ�30���ʼ��һ�ζ�ʱ�䡣
//	LED_CTR = 2;                //LEDָʾ����
//	LED_L();
	time_scan_flag = 1;         //�ɼ����ˣ����¼��㻽�Ѵ�����
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
		else if ( USB_TEST_READ() == 1)    //USB���ϣ���������״̬��ֹͣ�����ɼ�������
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

			if (delay_time > 2000) //�ȶ�1���
			{
				if (usb_flag == 0)
				{
					usb_flag = 1;         //1ʱ��ʾ��USB�����

					//LED_CTR = 1;          //LEDָʾ������
					Set_System();             //����SHE,8MHZ��PLL.
					GPIO_Configuration();             //����LED,KEY,USB,USB��Դ�ܽš�
					spi_io_enable(1);
					Set_USBClock();     //����Ҫ�Ȱ���ʱ�ӱ�Ƶ��72MHZ
					USB_Init();
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
					NVIC_Config(1);

					CCR1_Val = 1000;
					TIMER_Init();
					//UART1_Init();

					SPI_FLASH_Init(1);  //4��Ƶ
					ds3231_io_init(1);
					fand_read_wirte_cont();  //����д��㡾ͷ������㡾β����
				}
				if (Out_Buffer[0] == 0x5a)  //USB�н�������
				{
					//led_hold_ms = 2000;   //����1��
					//LED_CTR = 3;
					USB_com();
				}
				uart_rxcont_sum = get_string(1, uart1_data);
				//if (uart_rxcont_sum > 2) //���ô���������
				//{
				//led_hold_ms = 2000;   //����1��
				//LED_CTR = 3;
				//uart_com();         //�������ã����ԡ�ֻҪ��USBʱ�����á�
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
		else if (scan_test_en == 1)           //�ɼ���������
		{
			//segment.display = SEG_DISPLAY_OFF;

			tim1_io_enable(1);
			HWRXD_EN_H();  //�򿪺����Դ
			uart2_io_enable(1);
			scan_flag = scan_test();   //�ɼ����ԡ�
			uart2_io_enable(0);
			HWRXD_EN_L();  //�رպ����Դ
			tim1_io_enable(0);

			if (scan_flag > 0)          //���Գɹ�
			{
				savedata_in();         //�洢�ɼ��������ڲ�FLASH;
				char scan_resault[2] =
				{ '0', '0' };
				seg_itoa(scan_flag - 1, scan_resault, 2, 10);
				segment.dig8[0] = ascii2seg_default(scan_resault[0]);
				segment.dig8[1] = ascii2seg_default(scan_resault[1]);
			}
			else //����ʧ��
			{
				//LED_CTR = 2; //����
				//LED_L();
				//segment.display = SEG_DISPLAY_ON;
				segment.dig8[0] = ascii2seg_default('E');
				segment.dig8[1] = ascii2seg_default('r');
			}

			if (key_test_flag == 1)  //���԰��������¹���־��
			{
				wakeup_flag = 1;
				key_test_flag = 0;    //���԰��������¹���־��
				//LED_CTR = 2;          //����
				//led_cont = 6;         //��˸3��
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
		else //�����ɼ�����״̬
		{
			delay_time = 0;
			if (usb_flag == 1)       //1ʱ��ʾ��USB���������ϵͳ���������á�
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
				usb_flag = 0;

				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
				IWDG_SetPrescaler(IWDG_Prescaler_32);
				IWDG_SetReload(0x001);          //2s 0x4e2  1s
				IWDG_ReloadCounter();
				WWDG_EN();
				return_main();   //������
				while (1)
					;         //���ﹷ����Ч��
			}
			//===================
			spi_io_enable(0);
			segment.display = SEG_DISPLAY_OFF;
			seg_flush(&segment, &seg_cfg);
			WWDG_WEI();
			PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI); //����STOPģʽ
			WWDG_WEI();
			//���Ѻ�HSI��ѡΪϵͳʱ�ӡ��ɼ�ʱΪ���¶�Ӱ����С��Ӧ��ΪHSEʱ�ӡ�
			//===================
			/*
			 if (led_cont > 0) //ͨ�������������LED��˸��
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
			if (read_s_cont == 0) //��ʱ��
			{
				ds3231_io_init(1);
				top_time[2] = iic_1_readbyte(0x01); //��
				ds3231_io_init(0);
				time_hex_10 = (top_time[2] >> 4) * 10 + (top_time[2] & 0x0f);

				if (old_time != top_time[2]
						&& (time_hex_10 % set_updata_time) == 0) //������ʱ���������,���ﲻ���ж��룬�п��ܴ���롣
				{
					Set_System();           //����SHE,8MHZ��PLL.
					old_time = top_time[2]; //ÿ�ηֱ仯���п��ܽ���һ�Ρ�

					scan_en = 1;              //�ɼ�����
					time_scan_flag = 1;       //ʱ�䵽�ˣ��ɼ��˱�־��
					spi_io_enable(1);       //дFLashҪ��ǰ���硣
				}
			}
			else
			{
				read_s_cont--;
			}
			//------------------------------------------------------------------
			if (scan_en == 1)      //�ɼ�����
			{
				tim1_io_enable(1); //38KHZ
				HWRXD_EN_H();  //�򿪺����Դ
				uart2_io_enable(1);
				scan_flag = scan_vaw();  //�ɼ�
				uart2_io_enable(0);
				HWRXD_EN_L();  //�رպ����Դ
				tim1_io_enable(0);

				if (scan_flag == 0)        //�ɼ��ɹ������³ɹ�һ����
				{
					ds3231_io_init(1);
					top_time[3] = iic_1_readbyte(0x02);
					top_time[4] = iic_1_readbyte(0x03);
					top_time[5] = iic_1_readbyte(0x04);
					top_time[6] = iic_1_readbyte(0x05);
					top_time[7] = iic_1_readbyte(0x06);
					ds3231_io_init(0);
					save_data_cont = 0;       //FLASHд����
					save_data_buf[save_data_cont++] = 0xa5;  //��д��־������š���ʱ�䡿�����ݡ�
					save_data_buf[save_data_cont++] = top_addr[1]; //���ַ,��ʱ�������������ַ����ֻ�ڲ���ʱ��ȡ�������Ի�λ�û����
					save_data_buf[save_data_cont++] = top_addr[2];
					save_data_buf[save_data_cont++] = top_addr[3];
					save_data_buf[save_data_cont++] = top_addr[4];
					save_data_buf[save_data_cont++] = top_addr[5];
					save_data_buf[save_data_cont++] = top_addr[6];
					save_data_buf[save_data_cont++] = top_time[7]; //��
					save_data_buf[save_data_cont++] = top_time[6];
					save_data_buf[save_data_cont++] = top_time[5];
					save_data_buf[save_data_cont++] = top_time[4];
					save_data_buf[save_data_cont++] = top_time[3];
					save_data_buf[save_data_cont++] = top_time[2];
					save_data_buf[save_data_cont++] = 0x00; //��
					//spi_io_enable(1);
					flash_wirte_data(save_data_buf);    //дFLASH����
					spi_io_enable(0);
				}
				else //�ɼ�ʧ�ܣ���ɼ����ԡ�
				{
					scan_test_en = 1; //�ɼ���������
				}
				WWDG_WEI();
				scan_en = 0;
			}
//			it_stat = EXTI_GetITStatus(EXTI_Line0);
			if (time_scan_flag == 1) //�ɼ����˾�Ҫ����ʱ��㡣
			{
				ds3231_io_init(1);
				top_time[1] = iic_1_readbyte(0x00); //��
				top_time[2] = iic_1_readbyte(0x01); //��
				ds3231_io_init(0);

				time_scont[0] = (top_time[1] >> 4) * 10 + (top_time[1] & 0x0f);
				time_scont[1] = (top_time[2] >> 4) * 10 + (top_time[2] & 0x0f);
				read_s_cont = set_updata_time
						- (time_scont[1] % set_updata_time);
				read_s_cont = read_s_cont * 60 - time_scont[0]; //������һ����ʱ��㣬read_s_cont��
				time_scan_flag = 0; //ʱ�䵽�ˣ��ɼ��˱�־��
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
	else if (scan_var > 4) //5��6żУ��2400
	{
		UART2_Init(2);
	}
	else if (scan_var > 2) //3��4��У��1200
	{
		UART2_Init(1);
	}
	else //1��2żУ��1200
	{
		UART2_Init(0);
	}

	save_data_cont = 14;  //FLASHд����
	for (i = save_data_cont; i < 64; i++) //ǰ14���ֽ�Ϊ����־1������ַ6���͡�ʱ��7��
	{
		save_data_buf[i] = 0xff; //�������������Ч��
	}

	for (i = 0; i < 18; i++)
	{
		if (scan_var == 1 || scan_var == 3 || scan_var == 5 || scan_var == 7) //��07����
		{
			put_string(2, &com_cmd07[i][1], com_cmd07[i][0]); //��07���� ����1������
		}
		else //��97����
		{
			put_string(2, &com_cmd97[i][1], com_cmd97[i][0]); //��97���� ����1������
		}
		t2_cont = 0;
		len = RXD_SUM;
		while (t2_cont <= 2000) //�Ȼظ�����ʱʱ��2000ms��
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
			if (len > 10) //�յ�����
			{
				for (j = 6; j < len - 8; j++)
				{
					if (scan_var == 1 || scan_var == 3 || scan_var == 5
							|| scan_var == 7) //��07����
					{
						if (buf[j] == 0x68 && buf[j + 1] == 0x91
								&& (buf[j + 2] == 7 || buf[j + 2] == 6))
						{
							sum = 0;
							sum_len = buf[j + 2] + 10;
							for (cont = 0; cont < sum_len; cont++)
							{
								sum += buf[j - 7 + cont]; //�ӵ�һ��0x68��ʼ�ۼӡ�����68���7
							}
							if (sum == buf[j - 7 + sum_len])  //У��
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
											+ 7 + k] - 0x33;  //��¼����07��Լ
								}
								flag = 1; //�ɹ���
								//success_count++;
								break;//�ɹ�������һ��
							}
						}
					}
					else //97��Լ������
					{
						if (buf[j] == 0x68 && buf[j + 1] == 0x81
								&& (buf[j + 2] == 5 || buf[j + 2] == 4))
						{
							sum = 0;
							sum_len = buf[j + 2] + 10;
							for (cont = 0; cont < sum_len; cont++)
							{
								sum += buf[j - 7 + cont]; //�ӵ�һ��0x68��ʼ�ۼӡ�����68���7
							}
							if (sum == buf[j - 7 + sum_len]) //У��
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
											+ 5 + k] - 0x33; //��¼����97��Լ
								}
								//------------------------------------------------------

								if (zuo_97to07[i] == 1) //���� ���ݸ��ں󣬵���ǰ��
								{
									save_data_buf[save_data_cont + 2] =
											save_data_buf[save_data_cont + 1]
													>> 4; //��2����ӽ�����
									save_data_buf[save_data_cont + 1] <<= 4;
									save_data_buf[save_data_cont + 1] +=
											save_data_buf[save_data_cont] >> 4;
									save_data_buf[save_data_cont] <<= 4;
								}
								else if (zuo_97to07[i] == 2) //��ѹ
								{
									save_data_buf[save_data_cont + 1] <<= 4; //��1���ĵ�4λ����4λ
									save_data_buf[save_data_cont + 1] +=
											save_data_buf[save_data_cont] >> 4; //��0���ĸ�4λ����1���ĵ�4λ��
									save_data_buf[save_data_cont] <<= 4; //��0���ĵ�4λ����0���ĸ�4λ
								}
								else if (zuo_97to07[i] == 3) //�޹�����
								{
									save_data_buf[save_data_cont + 2] =
											save_data_buf[save_data_cont + 1]; //��2����ӽ�����
									save_data_buf[save_data_cont + 1] =
											save_data_buf[save_data_cont];
									save_data_buf[save_data_cont] = 0x00;
								}

								//------------------------------------------------------
								flag = 1; //�ɹ���
								//success_count++;
								break;//�ɹ�������һ��
							}
						}
					}
				}
				break; //���յ���Ч�ظ�
			}
		}
		if (flag == 0) //����ɹ������򲻻�������״̬��
		//if (success_count == 0)
		{
			return 1; //��һ��һ��Ҫ�ɹ�����������ԡ�
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
	UART2_Init(0); //żУ��1200
	ret = test07();
	if (ret > 0)
		return ret;
	ret = test97();
	if (ret > 0)
		return ret;

	UART2_Init(2);  //żУ��2400
	ret = test07();
	if (ret > 0)
		return ret;
	ret = test97();
	if (ret > 0)
		return ret;

	UART2_Init(1);  //��У��1200
	ret = test07();
	if (ret > 0)
		return ret;
	ret = test97();
	if (ret > 0)
		return ret;

	UART2_Init(3);  //��У��2400
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
	//����ʱ��
	if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x08
			&& Out_Buffer[10] == 0xa5)
	{
		iic_1_sendbyte(0x0f, 0x00);  //ֹͣ32K���
		iic_1_sendbyte(0x07, 0x00);  //��ֹ���ӡ�
		iic_1_sendbyte(0x0e, 0x00);  //�������1HZ,��ֹ���������жϡ�
		iic_1_sendbyte(0x06, Out_Buffer[3]);  //��
		iic_1_sendbyte(0x05, Out_Buffer[4]);  //��
		iic_1_sendbyte(0x04, Out_Buffer[5]);  //��
		iic_1_sendbyte(0x03, Out_Buffer[6]);  //����
		iic_1_sendbyte(0x02, Out_Buffer[7]);  //ʱ
		iic_1_sendbyte(0x01, Out_Buffer[8]);  //��
		iic_1_sendbyte(0x00, Out_Buffer[9]);  //��

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
	//��ȡʱ��
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x09
			&& Out_Buffer[3] == 0xa5)
	{
		In_Buffer[0] = 0x5a;
		In_Buffer[1] = 0x02;
		In_Buffer[2] = 0x09;
		In_Buffer[3] = iic_1_readbyte(0x06); //��
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
	//���洢����
	else if (Out_Buffer[1] == 0x02 && Out_Buffer[2] == 0x0b
			&& Out_Buffer[3] == 0xa5)
	{
		if (read_data_cont == 0)
		{
			read_data_flag = flash_read_data(In_Buffer);     //������ Ӧ�ж�״̬���Ƿ��������
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
	//�����洢��
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
		return_main(); //1�������
	}
	//���òɼ�����Ƶ��
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
	//��ȡ����ϵ�ʱ��
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
		iic_1_sendbyte(0x0f, 0x00);  //ֹͣ32K���
		iic_1_sendbyte(0x07, 0x00);//��ֹ���ӡ�
		iic_1_sendbyte(0x0e, 0x00);//�������1HZ,��ֹ���������жϡ�
		iic_1_sendbyte(0x06, uart1_data[3]);//��
		iic_1_sendbyte(0x05, uart1_data[4]);//
		iic_1_sendbyte(0x04, uart1_data[5]);//
		iic_1_sendbyte(0x03, uart1_data[6]);//����
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
		uart1_data[9] = iic_1_readbyte(0x00);  //��
		uart1_data[10] = 0xa5;

		put_string(1, uart1_data, 11);
	}
	else if (uart1_data[0] == 0x5a && uart_rxcont_sum >= 4
			&& uart1_data[1] == 0x02 && uart1_data[2] == 0x0b
			&& uart1_data[3] == 0xa5)
	{
		if (read_data_cont == 0)
		{
			read_data_flag = flash_read_data(read_data);     //������ Ӧ�ж�״̬���Ƿ��������
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
		return_main(); //1�������
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
