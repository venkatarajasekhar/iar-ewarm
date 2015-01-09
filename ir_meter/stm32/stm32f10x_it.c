/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
 * File Name          : stm32f10x_it.c
 * Author             : MCD Application Team
 * Version            : V2.2.1
 * Date               : 09/22/2008
 * Description        : Main Interrupt Service Routines.
 *                      This file provides template for all exceptions handler
 *                      and peripherals interrupt service routine.
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
#include "stm32f10x_it.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include <stdint.h>

/******************************************************************************/
//#include "seg.h"
//extern volatile seg_type segment;
//extern volatile uint8_t seg_en;
//extern seg_config seg_cfg;
extern volatile uint8_t adc_en;
extern volatile uint8_t wakeup_flag;
extern volatile uint32_t shutdown_cd;
extern volatile uint8_t scan_en;
extern volatile uint8_t shutdown_en;
/******************************************************************************/

extern vu16 CCR1_Val;
u32 uart1_time = 0;
u32 uart2_time = 0;
u32 t3_cont = 0;
vu32 t2_cont;
vu32 led_hold_ms;
vu8 LED_CTR = 0;

vu8 U1_RxBuffer[RXD_SUM];
vu8 U1_RxCounter;
vu8 U1_RxData_Total;
vu8 U1_ReceiveData_end;
//vu8   U1_RxBuffer2[RXD_SUM];
vu8 U1_TxBuffer[TXD_SUM];
vu8 U1_TxCounter;
vu8 U1_TxData_Total;
vu8 U1_SendData_end;

vu8 U2_RxBuffer[RXD_SUM];
vu8 U2_RxCounter;
vu8 U2_RxData_Total;
vu8 U2_ReceiveData_end;
//vu8   U2_RxBuffer2[RXD_SUM];
vu8 U2_TxBuffer[TXD_SUM];
vu8 U2_TxCounter;
vu8 U2_TxData_Total;
vu8 U2_SendData_end;

vu8 scan_test_en; //采集测试允许
vu8 key_test_flag; //测试按键被按下过标志。
vu32 delay_time;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : NMIException
 * Description    : This function handles NMI exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NMIException(void)
{
}

/*******************************************************************************
 * Function Name  : HardFaultException
 * Description    : This function handles Hard Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void HardFaultException(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/*******************************************************************************
 * Function Name  : MemManageException
 * Description    : This function handles Memory Manage exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void MemManageException(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/*******************************************************************************
 * Function Name  : BusFaultException
 * Description    : This function handles Bus Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void BusFaultException(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/*******************************************************************************
 * Function Name  : UsageFaultException
 * Description    : This function handles Usage Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void UsageFaultException(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/*******************************************************************************
 * Function Name  : DebugMonitor
 * Description    : This function handles Debug Monitor exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DebugMonitor(void)
{
}

/*******************************************************************************
 * Function Name  : SVCHandler
 * Description    : This function handles SVCall exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SVCHandler(void)
{
}

/*******************************************************************************
 * Function Name  : PendSVC
 * Description    : This function handles PendSVC exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PendSVC(void)
{
}

/*******************************************************************************
 * Function Name  : SysTickHandler
 * Description    : This function handles SysTick Handler.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SysTickHandler(void)
{
}

/*******************************************************************************
 * Function Name  : WWDG_IRQHandler
 * Description    : This function handles WWDG interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : PVD_IRQHandler
 * Description    : This function handles PVD interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PVD_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TAMPER_IRQHandler
 * Description    : This function handles Tamper interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TAMPER_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : RTC_IRQHandler
 * Description    : This function handles RTC global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RTC_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : FLASH_IRQHandler
 * Description    : This function handles Flash interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void FLASH_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : RCC_IRQHandler
 * Description    : This function handles RCC interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RCC_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : EXTI0_IRQHandler
 * Description    : This function handles External interrupt Line 0 request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
		EXTI_ClearITPendingBit(EXTI_Line0);
}

/*******************************************************************************
 * Function Name  : EXTI1_IRQHandler
 * Description    : This function handles External interrupt Line 1 request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EXTI1_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : EXTI2_IRQHandler
 * Description    : This function handles External interrupt Line 2 request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EXTI2_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		//LED_CTR = 1;  //亮。
		//LED_H();
		shutdown_cd = 3000;
		//GPIO_ResetBits(GPIOC, GPIO_Pin_12);

		if (wakeup_flag == 1) {
			scan_test_en = 1;  //采集测试允许
			key_test_flag = 1;  //测试按键被按下过标志。
		} else {
			adc_en = 1;
		}

		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

/*******************************************************************************
 * Function Name  : EXTI3_IRQHandler
 * Description    : This function handles External interrupt Line 3 request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EXTI3_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : EXTI4_IRQHandler
 * Description    : This function handles External interrupt Line 4 request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EXTI4_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA1_Channel1_IRQHandler
 * Description    : This function handles DMA1 Channel 1 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA1_Channel2_IRQHandler
 * Description    : This function handles DMA1 Channel 2 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA1_Channel3_IRQHandler
 * Description    : This function handles DMA1 Channel 3 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA1_Channel4_IRQHandler
 * Description    : This function handles DMA1 Channel 4 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA1_Channel5_IRQHandler
 * Description    : This function handles DMA1 Channel 5 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA1_Channel6_IRQHandler
 * Description    : This function handles DMA1 Channel 6 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA1_Channel7_IRQHandler
 * Description    : This function handles DMA1 Channel 7 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : ADC1_2_IRQHandler
 * Description    : This function handles ADC1 and ADC2 global interrupts requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void ADC1_2_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : USB_HP_CAN_TX_IRQHandler
 * Description    : This function handles USB High Priority or CAN TX interrupts
 *                  requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : USB_LP_CAN_RX0_IRQHandler
 * Description    : This function handles USB Low Priority or CAN RX0 interrupts
 *                  requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
	USB_Istr();
}

/*******************************************************************************
 * Function Name  : CAN_RX1_IRQHandler
 * Description    : This function handles CAN RX1 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CAN_RX1_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : CAN_SCE_IRQHandler
 * Description    : This function handles CAN SCE interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CAN_SCE_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : EXTI9_5_IRQHandler
 * Description    : This function handles External lines 9 to 5 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EXTI9_5_IRQHandler(void)
{

}

/*******************************************************************************
 * Function Name  : TIM1_BRK_IRQHandler
 * Description    : This function handles TIM1 Break interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM1_UP_IRQHandler
 * Description    : This function handles TIM1 overflow and update interrupt
 *                  request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM1_UP_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM1_TRG_COM_IRQHandler
 * Description    : This function handles TIM1 Trigger and commutation interrupts
 *                  requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM1_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM1_CC_IRQHandler
 * Description    : This function handles TIM1 capture compare interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM2_IRQHandler
 * Description    : This function handles TIM2 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM2_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM3_IRQHandler
 * Description    : This function handles TIM3 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM3_IRQHandler(void)
{
	u16 capture = 0;                     //定时器当前时间记录
	//u8 i;

	if (shutdown_cd > 0 && TEST_KEY_READ() == 0)
		shutdown_cd--;

	if (shutdown_cd < 2000)
	{
		adc_en = 0;
		scan_en = 0;
		scan_test_en = 0;
		key_test_flag = 0;
		shutdown_en = 1;
	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{

		t2_cont++;
		t3_cont++;
		if (delay_time < 10000)
			delay_time++;
		//--------------------------------------------------------------------------
		if (uart1_time <= 1000)                      //最大计数到5000，然后停止，等待用户给定初始值。
		{
			uart1_time++;                           //系统心跳时间累加
			if (uart1_time == 20 && U1_RxCounter > 0)  //如果时间大于10ms，则认为数据接收完成
			{
				U1_ReceiveData_end = 1;             //接收完成，
				U1_RxData_Total = U1_RxCounter;     //接收数据总数
				U1_RxCounter = 0;
				//for(i=0; i<U1_RxData_Total; i++)  //可不开双缓冲
				//{
				//    U1_RxBuffer2[i] = U1_RxBuffer[i];
				//}
			}
		}
		if (uart2_time <= 1000)                      //最大计数到5000，然后停止，等待用户给定初始值。
		{
			uart2_time++;                           //系统心跳时间累加
			if (uart2_time == 100 && U2_RxCounter > 0)  //如果时间大于10ms，则认为数据接收完成
			{
				U2_ReceiveData_end = 1;             //接收完成，
				U2_RxData_Total = U2_RxCounter;     //接收数据总数
				U2_RxCounter = 0;
				//for(i=0; i<U2_RxData_Total; i++)  //可不开双缓冲
				//{
				//    U2_RxBuffer2[i] = U2_RxBuffer[i];
				//}
			}
		}
		//--------------------------------------------------------------------------
		/*
		if (LED_CTR == 1) //LED_CTR 1【亮】 2【灭】3【快闪1S】 3自由
		{
			LED_H();
		}
		if (LED_CTR == 2)
		{
			LED_L();
		}
		else if (LED_CTR == 3)
		{
			if (t3_cont >= 600)
			{
				t3_cont = 0;
				LED_H();
			}
			else if (t3_cont >= 300)
			{
				LED_L();
			}
		}
		//-------------------------
		if (led_hold_ms > 0)
		{
			led_hold_ms--;
		}
		else
		{
			if (LED_CTR == 3)
			{
				LED_CTR = 1;
			}
		}
		*/
		capture = TIM_GetCapture1(TIM3);
		TIM_SetCompare1(TIM3, capture + CCR1_Val);
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
}

/*******************************************************************************
 * Function Name  : TIM4_IRQHandler
 * Description    : This function handles TIM4 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM4_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : I2C1_EV_IRQHandler
 * Description    : This function handles I2C1 Event interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void I2C1_EV_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : I2C1_ER_IRQHandler
 * Description    : This function handles I2C1 Error interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : I2C2_EV_IRQHandler
 * Description    : This function handles I2C2 Event interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void I2C2_EV_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : I2C2_ER_IRQHandler
 * Description    : This function handles I2C2 Error interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void I2C2_ER_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : SPI1_IRQHandler
 * Description    : This function handles SPI1 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SPI1_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : SPI2_IRQHandler
 * Description    : This function handles SPI2 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SPI2_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : USART1_IRQHandler
 * Description    : This function handles USART1 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USART1_IRQHandler(void)
{
	u8 rxd;
	u8 sum;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rxd = USART_ReceiveData(USART1);
		uart1_time = 0;

		U1_ReceiveData_end = 0;
		U1_RxBuffer[U1_RxCounter++] = rxd;
		if (U1_RxCounter >= RXD_SUM)
		{
			U1_RxCounter = 0;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE); //清接收中断。
	}
	if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
		//--------------------------------------------------------------------------
		if (U1_SendData_end == 0)
		{
			sum = U1_TxData_Total;
			if (U1_TxCounter < sum)                      //没发完
			{
				USART_SendData(USART1, U1_TxBuffer[U1_TxCounter++]);  //发送数据
			}
			else
			{
				U1_SendData_end = 1;                                  //数据发送结束
			}
		}
		//--------------------------------------------------------------------------
		USART_ClearITPendingBit(USART1, USART_IT_TC);
	}
}

/*******************************************************************************
 * Function Name  : USART2_IRQHandler
 * Description    : This function handles USART2 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USART2_IRQHandler(void)
{
	u8 rxd;
	u8 sum;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		if (U2_SendData_end == 1)
		{
			rxd = USART_ReceiveData(USART2);
			uart2_time = 0;

			U2_ReceiveData_end = 0;
			U2_RxBuffer[U2_RxCounter++] = rxd;
			if (U2_RxCounter >= RXD_SUM)
			{
				U2_RxCounter = 0;
			}
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE); //清接收中断。
	}
	if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)
	{
		//--------------------------------------------------------------------------
		if (U2_SendData_end == 0)
		{
			sum = U2_TxData_Total;
			if (U2_TxCounter < sum)                      //没发完
			{
				USART_SendData(USART2, U2_TxBuffer[U2_TxCounter++]);  //发送数据
			}
			else
			{
				U2_SendData_end = 1;                                  //数据发送结束
			}
		}
		//--------------------------------------------------------------------------
		USART_ClearITPendingBit(USART2, USART_IT_TC);
	}
}
/*******************************************************************************
 * Function Name  : USART3_IRQHandler
 * Description    : This function handles USART3 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USART3_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : EXTI15_10_IRQHandler
 * Description    : This function handles External lines 15 to 10 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : RTCAlarm_IRQHandler
 * Description    : This function handles RTC Alarm interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RTCAlarm_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : USBWakeUp_IRQHandler
 * Description    : This function handles USB WakeUp interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USBWakeUp_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM8_BRK_IRQHandler
 * Description    : This function handles TIM8 Break interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM8_BRK_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM8_UP_IRQHandler
 * Description    : This function handles TIM8 overflow and update interrupt
 *                  request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM8_UP_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM8_TRG_COM_IRQHandler
 * Description    : This function handles TIM8 Trigger and commutation interrupts
 *                  requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM8_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM8_CC_IRQHandler
 * Description    : This function handles TIM8 capture compare interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM8_CC_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : ADC3_IRQHandler
 * Description    : This function handles ADC3 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void ADC3_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : FSMC_IRQHandler
 * Description    : This function handles FSMC global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void FSMC_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : SDIO_IRQHandler
 * Description    : This function handles SDIO global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SDIO_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM5_IRQHandler
 * Description    : This function handles TIM5 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM5_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : SPI3_IRQHandler
 * Description    : This function handles SPI3 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SPI3_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : UART4_IRQHandler
 * Description    : This function handles UART4 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void UART4_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : UART5_IRQHandler
 * Description    : This function handles UART5 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void UART5_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM6_IRQHandler
 * Description    : This function handles TIM6 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM6_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : TIM7_IRQHandler
 * Description    : This function handles TIM7 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM7_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA2_Channel1_IRQHandler
 * Description    : This function handles DMA2 Channel 1 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA2_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA2_Channel2_IRQHandler
 * Description    : This function handles DMA2 Channel 2 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA2_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA2_Channel3_IRQHandler
 * Description    : This function handles DMA2 Channel 3 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA2_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
 * Function Name  : DMA2_Channel4_5_IRQHandler
 * Description    : This function handles DMA2 Channel 4 and DMA2 Channel 5
 *                  interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DMA2_Channel4_5_IRQHandler(void)
{
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
