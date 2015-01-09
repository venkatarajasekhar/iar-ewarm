//

#include "platform_config.h"

#define F_CS                    GPIO_Pin_4
#define F_CLK                   GPIO_Pin_5
#define F_MISO                  GPIO_Pin_6
#define F_MOSI                  GPIO_Pin_7

#define GPIO_CS                 GPIOA
#define RCC_APB2Periph_GPIO_CS  RCC_APB2Periph_GPIOA
#define GPIO_Pin_CS             F_CS

#define SPI_FLASH_CS_LOW()      GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS)
#define SPI_FLASH_CS_HIGH()     GPIO_SetBits(GPIO_CS, GPIO_Pin_CS)
//------------------------------------------------------------------------------
#define ALL_MYSECTOR_SUM        131072                  //(8*1024*1024)/64=131072
#define ALL_SECTOR_SUM          2048                    //(8*1024*1024)/4096
#define ALL_BYTE_SUM            (ALL_SECTOR_SUM*4096)   //(8*1024*1024)/4096
u32 read_sector_cont;           //���������
u32 wirte_sector_cont;          //д�������
u32 read_byte_cont;             //���ֽڼ�����
u32 wirte_byte_cont;            //д�ֽڼ�����
u32 read_page_cont=0;           //���Զ�ȡ�ϴ��İ�������64�ֽ�һ������
extern u8   read_data[256];     //��ȡ���塣
//==============================================================================
/* Private typedef -----------------------------------------------------------*/
#define SPI_FLASH_PageSize      256

/* Private define ------------------------------------------------------------*/
#define WRITE      0x02  /* Write to Memory instruction */
#define WRSR       0x01  /* Write Status Register instruction */
#define WREN       0x06  /* Write enable instruction */

#define READ       0x03  /* Read from Memory instruction */
#define RDSR       0x05  /* Read Status Register instruction  */
#define RDID       0x9F  /* Read identification */
//#define SE         0xD8  /* block Erase instruction  4k */
#define SE         0x20  /* block Erase instruction  4k */
#define BE         0xC7  /* Bulk Erase instruction */

#define WIP_Flag   0x01  /* Write In Progress (WIP) flag */

#define Dummy_Byte 0xA5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_Init(u8 sudu)
{
  SPI_InitTypeDef  SPI_InitStructure;
  
  /* Enable SPI1 and GPIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIO_CS, ENABLE);
  SPI_Cmd(SPI1, DISABLE);
  /* SPI1 configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  if(sudu==1)
  {
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  }
  else
  {
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  }
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}
void spi_io_enable( u8 sudu )
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if(sudu==1)
  {
      W25_EN_H();               //�򿪴洢��Դ  
      /* Configure SPI1 pins: SCK, MISO and MOSI */
      GPIO_InitStructure.GPIO_Pin = F_CLK | F_MISO | F_MOSI;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* Configure I/O for Flash Chip select */
      GPIO_InitStructure.GPIO_Pin = F_CS;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_Init(GPIO_CS, &GPIO_InitStructure);

      /* Deselect the FLASH: Chip Select high */
      SPI_FLASH_CS_HIGH();
      
  }
  else
  {
      /* Configure SPI1 pins: SCK, MISO and MOSI */
      GPIO_InitStructure.GPIO_Pin = F_CLK | F_MISO | F_MOSI;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* Configure I/O for Flash Chip select */
      GPIO_InitStructure.GPIO_Pin = F_CS;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
      GPIO_Init(GPIO_CS, &GPIO_InitStructure);
      W25_EN_L();               //�رմ洢��Դ
  }
}
/*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase
* Description    : Erases the specified FLASH block 4k.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_SectorErase(u32 SectorAddr)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();
/*
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(0x39);
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  SPI_FLASH_CS_HIGH();
  SPI_FLASH_WaitForWriteEnd();
*/
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();
  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  SPI_FLASH_SendByte(SE);
  /* Send SectorAddr high nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BulkErase(void)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();

  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Bulk Erase instruction  */
  SPI_FLASH_SendByte(BE);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  /* Enable the write access to the FLASH */
  SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */
  SPI_FLASH_SendByte(WRITE);
  /* Send WriteAddr high nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;                     //��ǰдĿ���ַ����ʼ��ַΪһҳ���м�ĳλ�ã�
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */ //NumOfSingle <= SPI_FLASH_PageSize - (SPI_FLASH_PageSize - Addr)
      {                                                                                //NumOfSingle <= Addr;
        temp = NumOfSingle - count;                          ////��ǰдĿ���ַ����ʼ��ַΪһҳ���м�ĳλ�ã�����д���ҳʣ���ַ��ʣ����������

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);      //д��Ŀ���ַ��ʼ�Ĳ���ҳ��ַ
        WriteAddr +=  count;                                 //д��ַ����Ϊ��ҳ��ַ���ɱ�ҳ�ֽ���������
        pBuffer += count;                                    //д���ݻ�������ַ��Ӧ����

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);       //�ڵ������ַ���ӻ���������ȡʣ������д��
      }
      else
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);  //���������С����Ŀ���ַΪ��ʼ��ַ��ʣ���ַ������ֱ��д�����ݵ���ҳ��
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(READ);

  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
u32 SPI_FLASH_ReadID(void)
{
  u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(0x9F);

  /* Read a byte from the FLASH */
  Temp0 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp1 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp2 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_StartReadSequence
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_StartReadSequence(u32 ReadAddr)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(READ);

  /* Send the 24-bit address of the address to read from -----------------------*/
  /* Send ReadAddr high nibble address byte */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u8 SPI_FLASH_ReadByte(void)
{
  return (SPI_FLASH_SendByte(Dummy_Byte));
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u8 SPI_FLASH_SendByte(u8 byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendHalfWord
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
u16 SPI_FLASH_SendHalfWord(u16 HalfWord)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send Half Word through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, HalfWord);

  /* Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the Half Word read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(void)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Write Enable" instruction */
  SPI_FLASH_SendByte(WREN);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WaitForWriteEnd(void)
{
  u8 FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */
  SPI_FLASH_SendByte(RDSR);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte);

  }
  while ((FLASH_Status & WIP_Flag) == SET); /* Write in progress */

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}
//============================================================================== 
void  fand_read_wirte_cont( void ) //����д��㡾ͷ������㡾β����
{
  u32 i;
  u8  write_flag;
  u32 flash_adder1=0;
  u8  flag1=0xff;  //1��ʾд��0��ʾûд��
  u8  flag2=0xff;  //1��ʾд��0��ʾûд��,ff��Ч��
  
  read_sector_cont=0;
  wirte_sector_cont=0;
  flash_adder1 = 0;  //0����ַ
  SPI_FLASH_BufferRead( &write_flag, flash_adder1, 1 ); //����д״̬��־
  if( write_flag!=0xff ) //0��ַд��
  {
      flag1=1;
  }
  else //0��ַûд��
  {
      flag1=0;
  }
  //----------------------------------------------------------------------------
  for( i=1; i<ALL_SECTOR_SUM; i++)
  {
     WWDG_WEI();
     flash_adder1 = (i<<12);  //4096�ֽ�
     SPI_FLASH_BufferRead( &write_flag, flash_adder1, 1 ); //����д״̬��־
     if( write_flag!=0xff ) //��ʼ��д״̬�жϡ�
     {
       if( flag1==0 )
       {
         read_sector_cont=i; //ԭ��û���������ˣ�����㡣
         flag2 = 1;   //��ǰ��
         flag1 = 0xff;
       }
       if( flag2==0 )
       {
         read_sector_cont=i; //ԭ��û���������ˣ�����㡣
         break;
       }
     }
     else //��ʼ��ַûд��
     {
       if( flag1==1 )
       {
         wirte_sector_cont=i; //ԭ���У�����û�ˣ�д��㡣
         flag2 = 0;    //��ǰ��
         flag1 = 0xff;
       }
       if( flag2==1 )  //ԭ���У�����û��д��㡣
       {
         wirte_sector_cont=i; //ԭ���У�����û�ˣ�д��㡣
         break;
       }
     }
  }
  //----------------------------------------------------------------------------
  flash_adder1 = (ALL_SECTOR_SUM-1)<<12;  //0����ַ
  SPI_FLASH_BufferRead( &write_flag, flash_adder1, 1 ); //����д״̬��־
  if( write_flag==0xff ) //0��ַûд��
  {
      read_sector_cont = 0;
  }
  //----------------------------------------------------------------------------�ҵ����ڵ�дλ��
  if( wirte_sector_cont==read_sector_cont )
  {
      wirte_byte_cont = (wirte_sector_cont<<12);  //4096�ֽ� 
  }
  else
  {
      if( wirte_sector_cont==0 )        wirte_sector_cont = ALL_SECTOR_SUM-1; //0-(ALL_SECTOR_SUM-1)
      else                              wirte_sector_cont--;
      for(i=0; i<64; i++)
      {
          wirte_byte_cont = (wirte_sector_cont<<12)+(i*64);  //4096�ֽ�
          SPI_FLASH_BufferRead( &write_flag, wirte_byte_cont, 1 ); //����д״̬��־
          if( write_flag==0xff ) //0��ַûд��
          {
              break;
          }
      }
  }
  read_byte_cont = (read_sector_cont<<12);    //4096�ֽ�
  //----------------------------------------------------------------------------����������
  if(read_byte_cont<wirte_byte_cont)
  {
    read_page_cont = wirte_byte_cont - read_byte_cont;
  }
  else if(read_byte_cont>wirte_byte_cont)
  {
    read_page_cont = wirte_byte_cont +(ALL_BYTE_SUM - read_byte_cont);
  }
  else
  {
    read_page_cont=0;
  }
  read_page_cont = read_page_cont>>6; //����ɶ�ȡ�İ�����
}
//==============================================================================
void  flash_wirte_data( u8 *data1 )            //д����
{
  u8  write_flag;
  u32 flash_adder1=0;
  
  if( (wirte_byte_cont&0xfff)==0 )            //����ǿ��׵�ַ
  {
      SPI_FLASH_BufferRead( &write_flag, wirte_byte_cont, 1 ); //����������
      if( write_flag!=0xff )
      {
          WWDG_WEI();
          SPI_FLASH_SectorErase( wirte_byte_cont ); //���дָ����ָʾ��4K�顣
      }
      flash_adder1 = wirte_byte_cont+4096;
      if( flash_adder1>=ALL_BYTE_SUM )
      {
          flash_adder1 = 0;
      }
      SPI_FLASH_BufferRead( &write_flag, flash_adder1, 1 ); //����ǰ���������һ����
      if( write_flag!=0xff )
      {
          WWDG_WEI();
          SPI_FLASH_SectorErase( wirte_byte_cont+4096 ); //���дָ����ָʾ��4K�顣
      }
  }
  
  WWDG_WEI();
  SPI_FLASH_PageWrite( data1, wirte_byte_cont, 64 );//�洢64�ֽ�����,���ܳ���ҳ����256.
  wirte_byte_cont+=64;
  if(wirte_byte_cont>=ALL_BYTE_SUM)           //���ڵ�������ֽڿռ䣬�򷵻�0��ַ��
  {
      wirte_byte_cont=0;
  }
}
//==============================================================================
u8  flash_read_data( u8 *data_flash )  //������
{
  if( (read_byte_cont>>6)==(wirte_byte_cont>>6) )//��дָ����ͬҳ�����ʧ��
  {
      return( 0xff );
  }
  SPI_FLASH_BufferRead( data_flash, read_byte_cont, 64 ); //��������
  if(data_flash[0]==0xff)       //�ҵ����������λ�ã�ָ��Ҫ�ƶ�����һ������׵�ַ��
  {
      read_byte_cont>>=12;      //�Ƴ��������֡�����ֱ�Ӽ�4096,Ҫ��һ������׵�ַ.
      read_byte_cont+=1;
      read_byte_cont<<=12;      //��������һ������׵�ַ��
      if(read_byte_cont>=ALL_BYTE_SUM)
      {
        read_byte_cont=0;
      }
      if( (read_byte_cont>>6)==(wirte_byte_cont>>6) )//��дָ����ͬҳ�����ʧ��
      {
        return( 0xff );
      }
      SPI_FLASH_BufferRead( data_flash, read_byte_cont, 64 );//��������
  }
  read_byte_cont+=64;
  if(read_byte_cont>=ALL_BYTE_SUM)
  {
      read_byte_cont=0;
  }
  return( 0 );
}
//==============================================================================
void  Erase_all_sector( void )
{
  //����ÿ��Ź���ֻ��һ��һ�������
  u32 i;
  u32 flash_adder1=0;
  u8  write_flag;
  
  for( i=0; i<ALL_SECTOR_SUM; i++)      //�����ù��Ŀ顣
  {
      WWDG_WEI();
      flash_adder1 = (i<<12);           //ÿ����4k.
      SPI_FLASH_BufferRead( &write_flag, flash_adder1, 1 ); //����д״̬��־
      if( write_flag!=0xff )            //��ʼ��д״̬�жϡ�
      {
          SPI_FLASH_SectorErase( flash_adder1 );   
      }
  }
  //SPI_FLASH_BulkErase();
}
//==============================================================================









