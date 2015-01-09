/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
 * File Name          : hw_config.c
 * Author             : MCD Application Team
 * Version            : V2.2.1
 * Date               : 09/22/2008
 * Description        : Hardware Configuration & Setup
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#define SETDATAADDER          0X0800FC00       //�洢������ַ

ErrorStatus HSEStartUpStatus;              //�ⲿ����ʱ��״̬
vu32 FLASH_ID = 0;                  //�ⲿFLASH ID��
FLASH_Status flash_status;                  //�ڲ�FLASH���״̬
typedef void (*pFunction)(void);
pFunction Jump_To_Application;           //��ת����ָ��
u32 JumpAddress;                   //��ת��ַ

extern u8 set_updata_time;  //�ɼ����ڣ���λ�֣���Χ1-60�֡�����Ϊ0��
extern u8 scan_var; //1 [1200 07] 2 [1200 97]  3 [2400 07] 4 [2400 97] ����������Ϊ��Ч��
extern u8 batt_time[7];     //��������ʱ���룬����ϵ�ʱ�䡣
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//==============================================================================
void savedata_out(void)
{
	u8 progdata1[32];
	u8 i = 0;
	u32 DataAddress;    //����ڲ�FLASH��ַ��
	u32 *progadder;
	u8 *data1;

	DataAddress = (u32) SETDATAADDER;
	progadder = (u32 *) progdata1;
	for (i = 0; i < 8; i++)
	{
		*progadder = *((vu32*) DataAddress);
		progadder++;
		DataAddress += 4;
	}
	//------------------------------------
	data1 = progdata1;
	if (data1[0] > 0 && data1[0] < 61)
	{
		set_updata_time = data1[0]; //�ɼ����ڣ���λ�֣���Χ1-60�֡�����Ϊ0��
	}
	data1 += 1;
	if (data1[0] > 0 && data1[0] < 5)
	{
		scan_var = data1[0]; //1 [1200 07] 2 [1200 97]  3 [2400 07] 4 [2400 97] ����������Ϊ��Ч��
	}
	data1 += 1;
}
//==============================================================================
void savedata_in(void)
{
	u8 progdata1[32];
	u32 progdata;                               //��������ݻ���
	u32 DataAddress;                            //����ڲ�FLASH��ַ��
	u8 i = 0;
	u8 *data1;

	data1 = progdata1;
	data1[0] = set_updata_time;
	data1 += 1;
	data1[0] = scan_var;
	data1 += 1;

	progdata = (u32) progdata1;
	DataAddress = SETDATAADDER;
	//WWDG_WEI();

	FLASH_Unlock();                                                 //��FLASHд������
	flash_status = FLASH_ErasePage(DataAddress);                   //���һ��ҳ
	if ((*(u32*) DataAddress == 0xffffffff) && (flash_status == FLASH_COMPLETE))
	{
		for (i = 0; i < 8; i++)
		{
			//WWDG_WEI();
			FLASH_Unlock();                                         //��FLASHд������
			flash_status = FLASH_ProgramWord(DataAddress, *(u32*) progdata); //����ڲ�FLASH,��д��
			if ((*(u32*) DataAddress == *(u32*) progdata)
					&& (flash_status == FLASH_COMPLETE))
			{
				DataAddress += 4;                                    //����Ŀ�ĵ�ַ��
				progdata += 4;
			}
		}
	}

}

//==============================================================================
void mem_cpy(u8 *s1, u8 *s2, u16 n)
{
	u8 *su1 = s1;
	u8 *su2 = s2;

	for (; 0 < n; ++su1, ++su2, --n)
	{
		*su1 = *su2;
	}
}
//==============================================================================
/* find first occurrence of s2[] in s1[] */
u8 *str_str(u8 *s1, u8 *s2)
{
	u8 *sc1, *sc2;

	if (*s2 == '\0')
	{
		return ((u8 *) s1);
	}
	for (; (s1 = str_chr(s1, *s2)) != 0; ++s1)
	{
		for (sc1 = s1, sc2 = s2;;)
		{
			if (*++sc2 == '\0')
			{
				return ((u8 *) s1);
			}
			else if (*++sc1 != *sc2)
			{
				break;
			}
		}
	}
	return (0);
}
//------------------------------------------------------------------------------
/* find first occurrence of c in char s[] */
u8 *str_chr(u8 *s, u8 c)
{
	u8 ch = c;

	for (; *s != ch; ++s)
	{
		if (*s == '\0')
		{
			return (0);
		}
	}
	return ((u8 *) s);
}
//==============================================================================
void return_main(void)
{
	JumpAddress = *(vu32*) (0x08000004);                  //�ڶ�����Ϊ�������
	Jump_To_Application = (pFunction) JumpAddress;        //��ת��ַ��ֵ
	__MSR_MSP(*(vu32*) 0x08000000);
	Jump_To_Application();
}
//==============================================================================
/*******************************************************************************
 * Function Name  : Set_System
 * Description    : Configures Main system clocks & power.
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void Set_System(void)
{
	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/

	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	RCC_LSICmd(ENABLE);
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

		/* Wait till HSE is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x04)
		{
		}
	}
	else
	{
		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

		/* Wait till HSE is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x00)
		{
		}
	}

	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1
					| RCC_APB2Periph_AFIO , ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_USART2,
			ENABLE);
	//----------------------------------------------------------------------------
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);
	BKP_TamperPinCmd(DISABLE);

	PWR_WakeUpPinCmd(DISABLE); //WEKUP�ű����������أ������øùܽŵ��жϹ���ʵ�ֻ��ѡ�
}

/*******************************************************************************
 * Function Name  : Set_USBClock
 * Description    : Configures USB Clock input (48MHz).
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_USBClock(void)
{
	/* On STICE the PLL output clock is fixed to 72 MHz */
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

	/* Enable PLL */
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while (RCC_GetSYSCLKSource() != 0x08)
	{
	}

	/* Select USBCLK source */
	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

	/* Enable USB clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
 * Function Name  : Enter_LowPowerMode.
 * Description    : Power-off system clocks and power while entering suspend mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Enter_LowPowerMode(void)
{
	/* Set the device state to suspend */
	bDeviceState = SUSPENDED;
}

/*******************************************************************************
 * Function Name  : Leave_LowPowerMode.
 * Description    : Restores system clocks and power while exiting suspend mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Leave_LowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	/* Set the device state to the correct state */
	if (pInfo->Current_Configuration != 0)
	{
		/* Device configured */
		bDeviceState = CONFIGURED;
	}
	else
	{
		bDeviceState = ATTACHED;
	}
}

/*******************************************************************************
 * Function Name  : NVIC_Config.
 * Description    : Configures the USB interrupts.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void NVIC_Config(u8 hl)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;

	if (hl == 1)
	{
		//USB_LP_CAN_RX0_IRQHandler �жϺ����д���
		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		//------------------------------------------------------------------------------
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //������ռ���ȼ�
		//----------------------------------------------------------------------------
		// Enable the USART1 Interrupt
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//----------------------------------------------------------------------------
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//----------------------------------------------------------------------------

	/* Enable the EXTI0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure Key Button EXTI Line to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	//----------------------------------------------------------------------------

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQChannel;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
 * Function Name  : USB_Cable_Config.
 * Description    : Software Connection/Disconnection of USB Cable.
 * Input          : NewState: new state.
 * Output         : None.
 * Return         : None
 *******************************************************************************/
void USB_Cable_Config(FunctionalState NewState)
{
	if (NewState != DISABLE)
	{
		//GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
		GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
	}
	else
	{
		//GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
		GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
	}
}

/*******************************************************************************
 * Function Name  : GPIO_Configuration
 * Description    : Configures the different GPIO ports.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_WKUP, ENABLE);
	GPIO_InitStructure.GPIO_Pin = WKUP_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(WKUP, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_HWRXD_EN, ENABLE);
	GPIO_InitStructure.GPIO_Pin = HWRXD_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(HWRXD_EN, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_USBDA_EN, ENABLE);
	GPIO_InitStructure.GPIO_Pin = USBDA_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USBDA_EN, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_USBDB_EN, ENABLE);
	GPIO_InitStructure.GPIO_Pin = USBDB_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USBDB_EN, &GPIO_InitStructure);

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_LED, ENABLE);
	//GPIO_InitStructure.GPIO_Pin = LED_LAMP_PIN;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_Init(LED_LAMP, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_W25_EN, ENABLE);
	GPIO_InitStructure.GPIO_Pin = W25_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W25_EN, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_KEY, ENABLE);
	GPIO_InitStructure.GPIO_Pin = TEST_KEY_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(TEST_KEY, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);
	GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_USB_TEST, ENABLE);
	GPIO_InitStructure.GPIO_Pin = USB_TEST_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USB_TEST, &GPIO_InitStructure);

	/**************************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_10);

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC, GPIO_Pin_12);

}
/*******************************************************************************
 * Function Name  : Get_SerialNum.
 * Description    : Create the serial number string descriptor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Get_SerialNum(void)
{
	u32 Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(u32*) (0x1FFFF7E8);
	Device_Serial1 = *(u32*) (0x1FFFF7EC);
	Device_Serial2 = *(u32*) (0x1FFFF7F0);

	if (Device_Serial0 != 0)
	{
		CustomHID_StringSerial[2] = (u8) (Device_Serial0 & 0x000000FF);
		CustomHID_StringSerial[4] = (u8) ((Device_Serial0 & 0x0000FF00) >> 8);
		CustomHID_StringSerial[6] = (u8) ((Device_Serial0 & 0x00FF0000) >> 16);
		CustomHID_StringSerial[8] = (u8) ((Device_Serial0 & 0xFF000000) >> 24);

		CustomHID_StringSerial[10] = (u8) (Device_Serial1 & 0x000000FF);
		CustomHID_StringSerial[12] = (u8) ((Device_Serial1 & 0x0000FF00) >> 8);
		CustomHID_StringSerial[14] = (u8) ((Device_Serial1 & 0x00FF0000) >> 16);
		CustomHID_StringSerial[16] = (u8) ((Device_Serial1 & 0xFF000000) >> 24);

		CustomHID_StringSerial[18] = (u8) (Device_Serial2 & 0x000000FF);
		CustomHID_StringSerial[20] = (u8) ((Device_Serial2 & 0x0000FF00) >> 8);
		CustomHID_StringSerial[22] = (u8) ((Device_Serial2 & 0x00FF0000) >> 16);
		CustomHID_StringSerial[24] = (u8) ((Device_Serial2 & 0xFF000000) >> 24);
	}
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
