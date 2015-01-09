
void  ds3231_io_init( u8 hl );
void  iic_set_input( void );
void  iic_set_output( void );
void  delayms( vu8 n);
void  delayus( vu8 n);
void  iic_1_start( void );
void  iic_1_stop( void );
u8    iic_1_send( u8 data );
u8    iic_1_read( u8 nack );
void  iic_1_sendbyte( u8 addr,u8 data );
u8    iic_1_readbyte( u8 addr );
void  iic_1_send24(u8 addr,u8 data);
u8    iic_1_read24( u8 addr );
