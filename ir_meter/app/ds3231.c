
//==============================================================================
#include "platform_config.h"

#define ds3231_sda            GPIO_Pin_7
#define ds3231_scl            GPIO_Pin_6
#define ds3231_rst            GPIO_Pin_9

#define    IIC1_OUT     iic_set_output()                        //IIC�������״̬
#define    IIC1_IN      iic_set_input()                         //IIC��������״̬

#define    IIC1_CLK1    do{GPIO_SetBits(GPIOB, ds3231_scl);delayus(4); }while(0)        //ʱ������1
#define    IIC1_CLK0    do{GPIO_ResetBits(GPIOB, ds3231_scl);delayus(4); }while(0)       //ʱ������0

#define    IIC1_IO1     do{GPIO_SetBits(GPIOB, ds3231_sda);delayus(4); }while(0)         //��������1
#define    IIC1_IO0     do{GPIO_ResetBits(GPIOB, ds3231_sda);delayus(4); }while(0)       //��������0

#define    IIC1_RST1    do{GPIO_SetBits(GPIOB, ds3231_rst);delayus(4); }while(0)
#define    IIC1_RST0    do{GPIO_ResetBits(GPIOB, ds3231_rst);delayus(4); }while(0)

#define    IIC1_IOIN    GPIO_ReadInputDataBit(GPIOB, ds3231_sda)       //����������

//==============================================================================
//�������ƣ�MAIN
//��ڲ�������
//���ڲ�������
//������Ա����ͷ�㣬20070521
//���������������к���
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
//�������ƣ�delayms
//��ڲ�����N��ʱʱ�䣬��λms
//���ڲ�������
//������Ա����ͷ�㣬20070521
//����������ͨ����ʱ����
//==============================================================================
void       delayms( vu8 n)
{
  vu8  i;
  while(n--){ for(i=1;i>0;i--); }  //MEGA16L��1MHZ����Ӧ��ʱ����delayms(1)=100US
}
//------------------------------------------------------------------------------
void       delayus( vu8 n)                       
{
  while(n--);                      //MEGA16L��1MHZ����Ӧ��ʱ����delayus(0)=17US
}
//==============================================================================
//�������ƣ�iic_1_start
//��ڲ�������
//���ڲ�������
//������Ա����ͷ�㣬20070521
//����������IIC������������
//==============================================================================																			
void    iic_1_start( void )
{
  IIC1_OUT;                                     //�������״̬
  IIC1_CLK0;                                    
  IIC1_IO1;
  IIC1_CLK1;
  IIC1_IO0;
  IIC1_CLK0;
}
//==============================================================================
//�������ƣ�iic_1_stop
//��ڲ�������
//���ڲ�������
//������Ա����ͷ�㣬20070521
//����������IIC���߽�������
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
//�������ƣ�iic_1_send
//��ڲ�����DATA���͵��ֽ�����
//���ڲ�����0Ӧ����Ч����0Ӧ�����
//������Ա����ͷ�㣬20070521
//����������IIC�����ֽڷ��ͺ���
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
  IIC1_IO1;            //51ϵͳ��Ҫ�Ӵ˾�

  IIC1_IN;             //��������״̬
  IIC1_CLK1;
  i = IIC1_IOIN;       //��ȡӦ��λ
  IIC1_CLK0;
  IIC1_OUT;            //�������״̬
  return(i);  
}
//==============================================================================
//�������ƣ�iic_1_read
//��ڲ�����0Ӧ����Ч��1��Ӧ��λ��Ч
//���ڲ�����data���յ�����
//������Ա����ͷ�㣬20070521
//����������IIC�����ֽڽ��պ���
//==============================================================================
u8    iic_1_read( u8 nack )
{
  u8 i,data;
  data=0;
  IIC1_IO1;

  IIC1_IN;
  for(i=0;i<8;i++)
  {
      IIC1_CLK1;                         //51ϵͳ��Ҫ�Ӵ˾�
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
//�������ƣ�iic_1_sendbyte
//��ڲ�����ADDRĿ���ַ��DATAд�������
//���ڲ�������
//������Ա����ͷ�㣬20070521
//����������IIC������Ŀ���ַдһ�ֽ����ݺ���
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
//�������ƣ�iic_1_readbyte
//��ڲ�����ADDRĿ���ַ
//���ڲ�����DATAд�������
//������Ա����ͷ�㣬20070521
//����������IIC���߶�ȡĿ���ַ��һ�ֽ����ݺ���
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
//end�ļ�����
//==============================================================================
