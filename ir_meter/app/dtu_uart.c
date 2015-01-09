#include "platform_config.h"

extern vu8 U1_RxBuffer[RXD_SUM];
extern vu8 U1_RxCounter;
extern vu8 U1_RxData_Total;
extern vu8 U1_ReceiveData_end;
//extern vu8   U1_RxBuffer2[RXD_SUM];
extern vu8 U1_TxBuffer[TXD_SUM];
extern vu8 U1_TxCounter;
extern vu8 U1_TxData_Total;
extern vu8 U1_SendData_end;

extern vu8 U2_RxBuffer[RXD_SUM];
extern vu8 U2_RxCounter;
extern vu8 U2_RxData_Total;
extern vu8 U2_ReceiveData_end;
//extern vu8   U2_RxBuffer2[RXD_SUM];
extern vu8 U2_TxBuffer[TXD_SUM];
extern vu8 U2_TxCounter;
extern vu8 U2_TxData_Total;
extern vu8 U2_SendData_end;

//==============================================================================
#if 0
void UART1_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_Cmd(USART1, DISABLE);
	//----------------------------------------------------------------------------
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	/* Configure USART1 */
	USART_Init(USART1, &USART_InitStructure);
	//----------------------------------------------------------------------------
	/* Enable USART1 Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        //设置M23时收发允许，注册和监控时允许。
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	//----------------------------------------------------------------------------
	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);    //
	//----------------------------------------------------------------------------
}
#endif
void UART2_Init(u8 u2_bps)
{
	USART_InitTypeDef USART_InitStructure;

	USART_Cmd(USART2, DISABLE);
	//----------------------------------------------------------------------------
	if (u2_bps == 0)
	{
		//偶校验1200
		USART_InitStructure.USART_BaudRate = 1200;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		USART_InitStructure.USART_Parity = USART_Parity_Even;
	}
	else if (u2_bps == 1)
	{
		//无校验1200
		USART_InitStructure.USART_BaudRate = 1200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_Parity = USART_Parity_No;
	}
	else if (u2_bps == 2)
	{
		//偶校验2400
		USART_InitStructure.USART_BaudRate = 2400;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		USART_InitStructure.USART_Parity = USART_Parity_Even;
	}
	else // u2_bps == 3 无校验 2400
	{
		USART_InitStructure.USART_BaudRate = 2400;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_Parity = USART_Parity_No;
	}
	//USART_InitStructure.USART_BaudRate = 1200;
	//USART_InitStructure.USART_WordLength = USART_WordLength_8b;//USART_WordLength_9b;
	//USART_InitStructure.USART_Parity = USART_Parity_No;//USART_Parity_Even;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	/* Configure USART2 */
	USART_Init(USART2, &USART_InitStructure);
	//----------------------------------------------------------------------------
	/* Enable USART1 Receive and Transmit interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	//----------------------------------------------------------------------------
	/* Enable the USART1 */
	USART_Cmd(USART2, ENABLE);    //
	//----------------------------------------------------------------------------
}
//==============================================================================
void uart2_io_enable(u8 hl)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	if (hl == 1)
	{
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else
	{
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
}
//==============================================================================
void init_rx_buf(u8 port)
{
	if (port == 1)
	{
		U1_RxCounter = 0;                //接收数据计数器
		U1_ReceiveData_end = 0;          //接收进行中
	}
	else
	{
		U2_RxCounter = 0;                //接收数据计数器
		U2_ReceiveData_end = 0;          //接收进行中
	}
}
//==============================================================================
u8 get_string(u8 port, u8 *buf)
{
	u8 i;
	if (port == 1)
	{
		if (U1_ReceiveData_end == 1 && U1_RxData_Total < RXD_SUM)      //接收完成，
		{
			for (i = 0; i < U1_RxData_Total; i++)
			{
				buf[i] = U1_RxBuffer[i];
			}
			U1_RxData_Total = 0;
			U1_ReceiveData_end = 0;        //接收进行中
			return i;
		}
		return 0;
	}
	else
	{
		if (U2_ReceiveData_end == 1 && U2_RxData_Total < RXD_SUM)      //接收完成，
		{
			for (i = 0; i < U2_RxData_Total; i++)
			{
				buf[i] = U2_RxBuffer[i];
			}
			U2_RxData_Total = 0;
			U2_ReceiveData_end = 0;        //接收进行中
			return i;
		}
		return 0;
	}
}
//==============================================================================
void put_string(u8 port, u8 *pDrc, u8 strlen)
{
	u8 i;
	if (port == 1)
	{
		U1_SendData_end = 1;
		U1_TxCounter = 0;                                         //发送计数器清
		U1_TxData_Total = strlen;                                 //传输总长度
		if (U1_TxData_Total <= TXD_SUM)
		{
			for (i = 0; i < U1_TxData_Total; i++)
			{
				U1_TxBuffer[i] = pDrc[i];
			}
			U1_SendData_end = 0;                                 //发送进行中，等待发送完成。
			USART_SendData(USART1, U1_TxBuffer[U1_TxCounter++]); //发送第一个字节，启动发送中断。
		}
	}
	else
	{
		U2_SendData_end = 1;
		U2_TxCounter = 0;                                         //发送计数器清
		U2_TxData_Total = strlen;                                 //传输总长度
		if (U2_TxData_Total <= TXD_SUM)
		{
			for (i = 0; i < U2_TxData_Total; i++)
			{
				U2_TxBuffer[i] = pDrc[i];
			}
			U2_SendData_end = 0;                                 //发送进行中，等待发送完成。
			USART_SendData(USART2, U2_TxBuffer[U2_TxCounter++]); //发送第一个字节，启动发送中断。
		}
	}
}
//==============================================================================

