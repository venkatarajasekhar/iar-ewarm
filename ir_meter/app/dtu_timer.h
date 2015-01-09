#ifndef __DTU_TIMER_H
#define __DTU_TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_tim.h"

/* Exported functions ------------------------------------------------------- */
void TIMER_Init(void);
void tim1_io_enable( u8 en);
void delay_ms( u16 n );
void delay_us( u16 n );

#endif