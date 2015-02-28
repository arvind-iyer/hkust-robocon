#include "gyro.h"	 
#include "flash.h"

volatile s16 curr_ang_vel = 0;
volatile s16 prev_ang_vel = 0;
volatile s32 gyro_angle = 0;
volatile s16 curr_real_angle = 0;
volatile s16 prev_real_angle = 0;	
s16 gyro_cal_result = 0;
u8 gyro_state = 0;

volatile s32 sim1_angle = 0;
volatile s32 sim2_angle = 0;
volatile u8 sim_factor = 0;
volatile s32 sim_before = 0;
volatile s32 sim_now = 0;
volatile s32 sim_angle = 0;
volatile s16 real_angle =0;
s16 sim_angle_off  =  0;

u16 spi_frame(u16 data )
{				  
	u16 temp;
	gyro_chip_select();	 	
	
	while (SPI_I2S_GetFlagStatus(GYRO_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData( GYRO_SPI , data );

	while (SPI_I2S_GetFlagStatus(GYRO_SPI, SPI_I2S_FLAG_RXNE) == RESET );	  	
	temp = SPI_I2S_ReceiveData(GYRO_SPI);  
	gyro_chip_deselect();

	_delay_us(10);	
    return temp;	
}

void adis_write(u8 addr, u16 data)
{		   	
	u16 cmd1, cmd2;	
	u8 address = ( ( addr & 0x3F ) | 0x80 ); 
	cmd1 = ( address << 8 ) | ( data >> 8 );
	spi_frame( cmd1 );

	cmd2 = ( (address-1) << 8 ) | ( data & 0x00FF);
	spi_frame( cmd2 );
}

u16 adis_read(u8 addr )
{				     
	u16 address = (0x3F & addr);
	address = address << 8;
	
    spi_frame(address); 
	address = spi_frame( address );
	return address;
}

void gyro_init(void)
{
 	gyro_spi_init();
	
    //Reset.....
    adis_write(GYRO_COMD,0x0080);		// sofware reset
	_delay_ms(50);

    //Factory Cal...
    adis_write(GYRO_COMD, 0x0002); 

    //Set Filter...
    adis_write(GYRO_SENS,0x0404);		// setting the Dynamic Range 320/sec
    adis_write(GYRO_SMPL,0x0001);		// set Internal Sample Rate 1.953 * ( 0x0001 & 0x2F + 1 ) = 3.906ms
    adis_write(GYRO_COMD,0x0008 );		// Auxiliary DAC data latch

	_delay_ms(100);
	
	gyro_state = 1;
}

void gyro_cal(void)
{
    s32 gyro_zero = 0;
    s32 gyro_zero_sum = 0;
    s16 gyro_zero_avg = 0;
    s16 gyro_zero_count = 0;
    s32 flash_gyro_val = read_flash(0);
    s16 tmp = 0, i = 0;
    u16 buf = 0;
	
	if (gyro_state == 0)
		return;
	
    printf("Flash: %d", read_flash(0));
    for (i = 0; i < 1024; i++) {
        if (i >= 256) {
            tmp = gyro_get_vel();
            gyro_zero_sum += tmp;
            ++gyro_zero_count;
            gyro_zero_avg = gyro_zero_sum / gyro_zero_count;
            if (!(i % 16)) {
              printf("cal zero: %d \n\r",tmp);
            }
          
            if (gyro_zero_count > 20) {
              if (Abs(gyro_zero_avg - flash_gyro_val) <= 5) {
                break;
              }
            }
        }
		_delay_ms(4);
    }

    
    printf("Gyro_zero_avg = %d", gyro_zero_avg);
    write_flash(0, gyro_zero_avg);
    
    printf("Flash: %d", read_flash(0));
    
    buf |= gyro_zero_avg * (-4);
    gyro_cal_result = buf;
	
    adis_write(GYRO_OFF, buf );
    adis_write(GYRO_COMD, 0x0008 );
	_delay_ms(100);
	
	gyro_state = 2;
}

void gyro_cal_short(void)
{
    s32 gyro_zero32 = 0;
    s16 gyro_zero = 0;
    s16 tmp = 0 , i =0;
    u16 buf = 0;
	s16 prev_zero = 0;
	
	if (gyro_state == 0)
		return;
	
    for (i = 0; i < 256; i++) {
        tmp = gyro_get_vel();
        if (Abs(tmp) > 12)
            return;
        gyro_zero32 += tmp;
		_delay_ms(4);
    }

    gyro_zero32 /= 64;
    gyro_zero32 = -gyro_zero32;
    gyro_zero = (s16)gyro_zero32;

    if (Abs(gyro_zero) > 1) {
        prev_zero = gyro_get_off();
        gyro_zero += prev_zero;
        buf |= gyro_zero;
        adis_write(GYRO_OFF, buf);
        adis_write(GYRO_COMD,0x0008);
		_delay_ms(100);
    }
	
	gyro_state = 2;
}

s16 gyro_get_off(void)
{					
    u16 buf = 0;
    s16 off = 0;
    buf = adis_read(GYRO_OFF);	
 
    if (buf & 0x0800 ) // 0b0000100000000000
        buf |= 0xF000; // 0b1111000000000000
    else
        buf &= 0x0FFF; // 0b0000111111111111;	
    off |=buf;
    return off;
}

s16 gyro_get_vel(void)
{		
    u16 buf = 0;
	s16 vel = 0;
    buf = adis_read(GYRO_VEL);
    if (buf & 0x2000 ) // 0b0010000000000000)
        buf |= 0xC000;  //0b1100000000000000
    else
        buf &= 0x3FFF; //0b0011111111111111;
    vel |=buf;

    return vel;
}

u16 gyro_get_angle(void)
{		  	 	
    u16 angle = 0;
    angle = adis_read(GYRO_ANGL) & 0x3FFF;   //0b0011111111111111;
    return angle;
}		  

void gyro_update(void){ 
	curr_ang_vel = gyro_get_vel();
	if ( curr_ang_vel < GYRO_ANG_VEL_TH && curr_ang_vel > -GYRO_ANG_VEL_TH) {
	    curr_ang_vel = 0;
	    if (sim_factor) {
			sim_factor = 0;
	    }
	}

	if (curr_ang_vel) {
		if (!sim_factor) {		// 1st time
			sim_before = sim_now;
		//	sim1_angle = (curr_ang_vel+prev_ang_vel) >> 1;
			sim1_angle = curr_ang_vel;
			sim2_angle = 0;
			sim_factor = 3;
		} else {
			sim1_angle += (sim_factor*prev_ang_vel+curr_ang_vel)/3;
			sim_factor = 4-sim_factor;
			sim2_angle += (sim_factor*prev_ang_vel+curr_ang_vel)/3;
		}
	
		sim_now = sim_before+(sim_factor == 3 ? sim1_angle : sim2_angle);
		sim_angle = sim_now/GYRO_SCALE ;
		
		sim_angle = sim_angle % 3600;
		if (sim_angle < 0)
			sim_angle +=3600;
		real_angle = sim_angle = (sim_angle!=0)?3600-sim_angle:0;
		real_angle = (real_angle + sim_angle_off ) % 3600;
		if( real_angle < 0 )
			real_angle += 3600;
	}

//	gyro_angle += (curr_ang_vel+prev_ang_vel) >> 1;
	gyro_angle += curr_ang_vel;
	curr_real_angle = gyro_angle/GYRO_SCALE; //divide the time constant

    prev_ang_vel = curr_ang_vel;
 //   prev_real_angle = curr_real_angle;
}  

u16 gyro_get_flash(void)
{	
	adis_read(GYRO_FLASH);   
	return adis_read(GYRO_FLASH);
}

u16 gyro_get_power(void)
{ 	
	adis_read(GYRO_POWER);
	return 1832*(adis_read(GYRO_POWER) & 0x0FFF);
}

u16 gyro_get_adc(void)				 
{	
	adis_read(GYRO_ADC);
	return 6105*(adis_read(GYRO_ADC) & 0x0FFF);
}

u16 gyro_get_temp(void)
{	
	adis_read(GYRO_TEMP);
	return 145*(adis_read(GYRO_TEMP) & 0x0FFF);
}

void gyro_chip_select( void ){
	GPIO_ResetBits( GYRO_GPIO , GYRO_PIN_NSS );	
}

void gyro_chip_deselect( void ){
	GPIO_SetBits( GYRO_GPIO , GYRO_PIN_NSS );
}

void gyro_spi_init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef IO_2;
	GPIO_InitTypeDef IO_1;    
	
	//rcc init
	RCC_APB2PeriphClockCmd( GYRO_CLK |  GYRO_GPIO_CLK, ENABLE );
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable , ENABLE); 	for SPI3 only

	//gpio init
	IO_1.GPIO_Pin = GYRO_PIN_SCK | GYRO_PIN_MOSI;
	IO_1.GPIO_Speed = GYRO_GPIO_SPEED;		
	IO_1.GPIO_Mode = GPIO_Mode_AF_PP;
	
	IO_2.GPIO_Pin = GYRO_PIN_MISO;
	IO_2.GPIO_Speed = GYRO_GPIO_SPEED;	
	IO_2.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GYRO_GPIO, &IO_1);
	GPIO_Init(GYRO_GPIO, &IO_2);
	
	IO_1.GPIO_Speed = GYRO_GPIO_SPEED;
	IO_1.GPIO_Pin = GYRO_PIN_NSS ;
	IO_1.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GYRO_GPIO, &IO_1);
	GPIO_SetBits( GYRO_GPIO , GYRO_PIN_NSS );
	
	//reset pin
	/*
		IO_1.GPIO_Speed = GYRO_GPIO_SPEED;
		IO_1.GPIO_Pin = GPIO_Pin_8;
		IO_1.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &IO_1);
		GPIO_Init(GPIOE ,&IO_1);
		GPIO_SetBits( GPIOB , GPIO_Pin_8 );
		GPIO_SetBits( GPIOE , GPIO_Pin_8 );	 	
	*/
	
	
	//spi init
	SPI_StructInit(&SPI_InitStructure );
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_BaudRatePrescaler = GYRO_BR_Prescaler;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	
	SPI_I2S_DeInit(GYRO_SPI); 
	SPI_Cmd(GYRO_SPI, DISABLE);  
	
	SPI_Init(GYRO_SPI, &SPI_InitStructure);
	SPI_Cmd(GYRO_SPI, ENABLE);
	SPI_SSOutputCmd( GYRO_SPI , DISABLE );
	SPI_CalculateCRC(GYRO_SPI, DISABLE); 	
	 										
	_delay_ms(100);

	/*
	GPIO_ResetBits( GPIOE , GPIO_Pin_8 );
	_delay_ms(150);
	GPIO_SetBits( GPIOE , GPIO_Pin_8 );
	*/	   
}



void set_angle( s16 angle ){
	
	//printf( "angle:%d, real_angle:%d , sim_angle:%d, sim_angle_off:%d " , angle , real_angle , sim_angle , sim_angle_off );
	sim_angle_off = angle - sim_angle;
	real_angle = angle ;
	//printf( "new_off:%d \r\n" , sim_angle_off);
}
