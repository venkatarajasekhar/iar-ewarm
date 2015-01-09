#ifndef __DTU_UART_H
#define __DTU_UART_H

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"

//==============================================================================
void uart2_io_enable(u8 hl);
void  UART1_Init(void);                              //����1��ʼ��
void  UART2_Init( u8 u2_bps );                              //����2��ʼ��
void  init_rx_buf( u8 port );
u8    get_string( u8 port, u8 *buf );
void  put_string(u8 port, u8 *pDrc, u8 strlen);
//==============================================================================
#endif


