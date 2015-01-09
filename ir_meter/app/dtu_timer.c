#include "platform_config.h"

//==============================================================================
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
vu16    CCR1_Val = 111;                            //比较寄存器1比较值为1000，8MHZ时对应1000/9=111
//==============================================================================
void TIMER_Init( void )
{
  TIM_DeInit(TIM2);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  //TIM1 used for PWM genration
  TIM_TimeBaseStructure.TIM_Prescaler = 0x00; // TIM2CLK = 72 MHz
  TIM_TimeBaseStructure.TIM_Period = 210; // 自动重装计数值
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  //TIM1's Channel2 in PWM1 mode
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 105;// Duty cycle: 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;  
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  //TIM_SetCompare1();
  /* ---------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 4, TIM2 counter clock = 7.2 MHz
    CC1 update rate = TIM2 counter clock / CCR1_Val = 146.48 Hz
  --------------------------------------------------------------- */
  TIM_DeInit( TIM3 );
  TIM_Cmd(TIM3, DISABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                    //自动重装计数值
  TIM_TimeBaseStructure.TIM_Prescaler = 35;                     //预分频值 (35+1)==36/36=1us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //时钟分频因子 //采样分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  //TIM_PrescalerConfig(TIM2, 35, TIM_PSCReloadMode_Immediate);   //这个是TIM_TimeBaseInit中预分频和装入设置。功能上重复，但可在计数时设置。
  //------------------------------------------------------------
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;           //定时模式
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;//禁止引脚输出
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;                     //1000*1uS = 1mS
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //这里用不到

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* TIM IT enable */
  TIM_ITConfig(TIM3, TIM_IT_CC1 , ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM3, ENABLE);           //开启时钟
}
void tim1_io_enable( u8 en) // tim2
{
  GPIO_InitTypeDef GPIO_InitStructure;
  if(en==1)
  {
      // Configure PA.08 as alternate function (TIM1_OC1)
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
      GPIO_InitStructure.GPIO_Pin = TIM1_OUTPUT_GPIO_PIN;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(TIM1_OUTPUT_GPIO_TYPE, &GPIO_InitStructure);
      /* TIM1 counter enable */
      TIM_Cmd(TIM2, ENABLE);
      /* TIM1 Main Output Enable */
      TIM_CtrlPWMOutputs(TIM2, ENABLE);
  }
  else
  {
      /* TIM1 counter enable */
      TIM_Cmd(TIM2, DISABLE);
      /* TIM1 Main Output Enable */
      TIM_CtrlPWMOutputs(TIM2, DISABLE);
      // Configure PA.08 as alternate function (TIM1_OC1)
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Pin = TIM1_OUTPUT_GPIO_PIN;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(TIM1_OUTPUT_GPIO_TYPE, &GPIO_InitStructure);
      GPIO_ResetBits(TIM1_OUTPUT_GPIO_TYPE, TIM1_OUTPUT_GPIO_PIN); //38K，输出低0。
  }
  
}
//==============================================================================
void  delay_ms( u16 timer)
{
  vu16 i;
  vu16 j;
  
  for( j=0; j<timer; j++ )
  {
    i=4000;
    while( i-- );
  }
}
//==============================================================================
void delay_us( u16 n )
{
  vu16 i;
  u16 j;
  
  j = n*3;
  for(i=0; i<j; i++);
  {
    //WWDG_WEI();
  }
}
//==============================================================================
