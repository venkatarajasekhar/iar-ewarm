
//==============================================================================
#include "platform_config.h"

#define ds3231_sda            GPIO_Pin_7
#define ds3231_scl            GPIO_Pin_6
#define ds3231_rst            GPIO_Pin_9

#define    IIC1_OUT     iic_set_output()                        //IIC总线输出状态
#define    IIC1_IN      iic_set_input()                         //IIC总线输入状态

#define    IIC1_CLK1    do{GPIO_SetBits(GPIOB, ds3231_scl);delayus(4); }while(0)        //时钟线置1
#define    IIC1_CLK0    do{GPIO_ResetBits(GPIOB, ds3231_scl);delayus(4); }while(0)       //时钟线清0

#define    IIC1_IO1     do{GPIO_SetBits(GPIOB, ds3231_sda);delayus(4); }while(0)         //数据线置1
#define    IIC1_IO0     do{GPIO_ResetBits(GPIOB, ds3231_sda);delayus(4); }while(0)       //数据线清0

#define    IIC1_RST1    do{GPIO_SetBits(GPIOB, ds3231_rst);delayus(4); }while(0)
#define    IIC1_RST0    do{GPIO_ResetBits(GPIOB, ds3231_rst);delayus(4); }while(0)

#define    IIC1_IOIN    GPIO_ReadInputDataBit(GPIOB, ds3231_sda)       //数据线输入

//==============================================================================
//函数名称：MAIN
//入口参数：无
//出口参数：无
//创建人员：胖头鱼，20070521
//功能描述：主运行函数
//==============================================================================
void iic_set_output( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  GPIO_InitStructure.GPIO_Pin   = ds3231_sda;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void iic_set_input( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  GPIO_InitStructure.GPIO_Pin   = ds3231_sda;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//------------------------------------------------------------------------------
void ds3231_io_init( u8 hl )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  if(hl==1)
  {
    GPIO_InitStructure.GPIO_Pin   = ds3231_scl;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    iic_set_output( );
  }
  else
  {
    GPIO_InitStructure.GPIO_Pin   = ds3231_sda|ds3231_scl;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;//GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    IIC1_IO1;
    IIC1_CLK1;
  }
}
//==============================================================================
//函数名称：delayms
//入口参数：N延时时间，单位ms
//出口参数：无
//创建人员：胖头鱼，20070521
//功能描述：通用延时函数
//==============================================================================
void       delayms( vu8 n)
{
  vu8  i;
  while(n--){ for(i=1;i>0;i--); }  //MEGA16L，1MHZ，对应延时程序delayms(1)=100US
}
//------------------------------------------------------------------------------
void       delayus( vu8 n)                       
{
  while(n--);                      //MEGA16L，1MHZ，对应延时程序delayus(0)=17US
}
//==============================================================================
//函数名称：iic_1_start
//入口参数：无
//出口参数：无
//创建人员：胖头鱼，20070521
//功能描述：IIC总线启动函数
//==============================================================================																			
void    iic_1_start( void )
{
  IIC1_OUT;                                     //总线输出状态
  IIC1_CLK0;                                    
  IIC1_IO1;
  IIC1_CLK1;
  IIC1_IO0;
  IIC1_CLK0;
}
//==============================================================================
//函数名称：iic_1_stop
//入口参数：无
//出口参数：无
//创建人员：胖头鱼，20070521
//功能描述：IIC总线结束函数
//==============================================================================
void    iic_1_stop( void )
{
  //IIC1_CLK0;                                    
  IIC1_IO0;
  IIC1_CLK1;
  IIC1_IO1;
  IIC1_CLK0;
}
//==============================================================================
//函数名称：iic_1_send
//入口参数：DATA发送的字节数据
//出口参数：0应答有效，非0应答错误
//创建人员：胖头鱼，20070521
//功能描述：IIC总线字节发送函数
//==============================================================================
u8    iic_1_send( u8 data )
{
  u8 i;

  for(i=0;i<8;i++)
  {
      if(data & 0x80)
        { IIC1_IO1; }
      else
        { IIC1_IO0; }

      IIC1_CLK1;
      data = data<<1;
      IIC1_CLK0;	
  }
  IIC1_IO1;            //51系统需要加此句

  IIC1_IN;             //总线输入状态
  IIC1_CLK1;
  i = IIC1_IOIN;       //读取应答位
  IIC1_CLK0;
  IIC1_OUT;            //总线输出状态
  return(i);  
}
//==============================================================================
//函数名称：iic_1_read
//入口参数：0应答有效，1非应答位有效
//出口参数：data接收的数据
//创建人员：胖头鱼，20070521
//功能描述：IIC总线字节接收函数
//==============================================================================
u8    iic_1_read( u8 nack )
{
  u8 i,data;
  data=0;
  IIC1_IO1;

  IIC1_IN;
  for(i=0;i<8;i++)
  {
      IIC1_CLK1;                         //51系统需要加此句
	  data = data<<1;
      if(IIC1_IOIN) 
         data = data | 0x01 ;
      IIC1_CLK0;
   }
   IIC1_OUT;
   if(nack)
     { IIC1_IO1; }
   else
     { IIC1_IO0; }	 
   IIC1_CLK1;	   
   IIC1_CLK0;
   IIC1_IO1;   
   return(data);  
}
//==============================================================================
//函数名称：iic_1_sendbyte
//入口参数：ADDR目标地址，DATA写入的数据
//出口参数：无
//创建人员：胖头鱼，20070521
//功能描述：IIC总线向目标地址写一字节数据函数
//==============================================================================
void     iic_1_sendbyte( u8 addr,u8 data )
{
 iic_1_start( );
 iic_1_send( 0xd0 );
 iic_1_send( addr );
 iic_1_send( data );
 iic_1_stop( );
 //delayms( 100 );
}
//==============================================================================
//函数名称：iic_1_readbyte
//入口参数：ADDR目标地址
//出口参数：DATA写入的数据
//创建人员：胖头鱼，20070521
//功能描述：IIC总线读取目标地址中一字节数据函数
//==============================================================================
u8    iic_1_readbyte( u8 addr )
{
 u8  data;
 iic_1_start( );
 iic_1_send( 0xd0 );
 iic_1_send( addr );
 iic_1_start( );
 iic_1_send( 0xd1 );
 data = iic_1_read( 1 );;;;;
 iic_1_stop( );
 //delayms( 100 ); 
 return(data); 
}
//==============================================================================
void       iic_1_send24(u8 addr,u8 data)
{
 iic_1_start( );
 iic_1_send( 0xa0 );
 iic_1_send( 0x00 );
 iic_1_send( addr );
 iic_1_send( data );
 iic_1_stop( );
 //delayms( 100 );
}
//------------------------------------------------------------------------------
u8      iic_1_read24( u8 addr )
{
 u8  data;
 iic_1_start( );
 iic_1_send( 0xa0 );
 iic_1_send( 0x00 ); 
 iic_1_send( addr );
 iic_1_start( );
 iic_1_send( 0xa1 );
 data = iic_1_read( 1 );
 iic_1_stop( );
 //delayms( 100 ); 
 return(data); 
}
//==============================================================================
//end文件结束
//==============================================================================
