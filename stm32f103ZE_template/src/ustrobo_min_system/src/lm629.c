#include "lm629.h"


// defined in lm629.h: #define LM629_REPORT_TWIN 		0xF0
u16 lm629_flag = 0;
u8 lm629_flag_ch[MAX_CHIP]; // Channel
s32 lm629_v[MAX_CHIP];		// velocity	 
u16 lm629_filter[MAX_CHIP][4];		 
u32 lm629_acc[MAX_CHIP];	  
u16 lm629_pe[MAX_CHIP];	 
u16 lm629_chip_available = 0;	 //max 16 chips
u8 dataP_dir=0;
u8 buf[4];
GPIO_InitTypeDef dataG;	//io init struc for data port
s32 lm629_read_pos (uc8 chip);
s32 lm629_read_pos_home (uc8 chip);
u8 lm629_pos_done(uc8 chip); 
u8 lm629_is_pe(uc8 chip); 
u8 lm629_bp_reach(uc8 chip);
s32 lm629_pos_backup[MAX_CHIP] = 0;

// LM629 low level access routines
// Define anything you have to write, probably Read/Write * Command/Data, Select/Reset chip
// e.g. static void write_command(u8 cmd);
// Note that there might not have a LM629 chip inserted. 
//   Pass the command as is without blocking if it's the case
static u8 ReadData(void);
static void WriteCommand(u8 cmd);
static void WriteData(u8 data);
static u32 U8ToSignedLong(uc8 * data);
static void CheckBusy(void);

/******************************************************************************
  Procedures
******************************************************************************/ 
// toggle data port input output direction
void set_dataP_dir(u8 mode){	//mode=1-->input
	dataG.GPIO_Pin = D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7;
	dataG.GPIO_Speed = GPIO_Speed_10MHz; 
	if(mode ){
		dataG.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		dataP_dir=1;
	}
	else{
		dataG.GPIO_Mode = GPIO_Mode_Out_PP;
		dataP_dir=0;
	}
	GPIO_Init(dataP, &dataG);
}

// read from data port 
u8 read_dataP(void){
	if(!dataP_dir)
		set_dataP_dir(1);  		
	return (u8)( GPIO_ReadInputDataBit(dataP, D0) |
		(GPIO_ReadInputDataBit(dataP, D1)<<1) |
		(GPIO_ReadInputDataBit(dataP, D2)<<2) |
		(GPIO_ReadInputDataBit(dataP, D3)<<3) |
		(GPIO_ReadInputDataBit(dataP, D4)<<4) |
		(GPIO_ReadInputDataBit(dataP, D5)<<5) |
		(GPIO_ReadInputDataBit(dataP, D6)<<6) |
		(GPIO_ReadInputDataBit(dataP, D7)<<7) );
}

// write to data port
void write_dataP(u8 data){
	if(dataP_dir)
		set_dataP_dir(0);
	GPIO_WriteBit(dataP, D0, (BitAction)(data&(1 << 0)));
	GPIO_WriteBit(dataP, D1, (BitAction)(data&(1 << 1)));
	GPIO_WriteBit(dataP, D2, (BitAction)(data&(1 << 2)));
	GPIO_WriteBit(dataP, D3, (BitAction)(data&(1 << 3)));
	GPIO_WriteBit(dataP, D4, (BitAction)(data&(1 << 4)));
	GPIO_WriteBit(dataP, D5, (BitAction)(data&(1 << 5)));
	GPIO_WriteBit(dataP, D6, (BitAction)(data&(1 << 6)));
	GPIO_WriteBit(dataP, D7, (BitAction)(data&(1 << 7)));
}

// check the busy flag and wait until the flag is cleared
static void CheckBusy(void){		     
	while(ReadStatus() & 1)	;
}

// select 629 chip
void SelectChip(u8 chip){
	GPIO_WriteBit(GPIOB, CS0, (BitAction)(chip & (1 << 0)));
	GPIO_WriteBit(GPIOB, CS1, (BitAction)(chip & (1 << 1)));
	GPIO_WriteBit(GPIOC, CS2, (BitAction)(chip & (1 << 2)));
	GPIO_WriteBit(GPIOC, CS3, (BitAction)(chip & (1 << 3)));
//	_delay_us(10);
	_delay_us(1);
}


/*ReadStatus
	To read the status of the motor.
	It will return a 8bit information in Hex mode.
	Bit Position  Function
	Bit 7			Motor Off
	Bit 6			BreakpointReached[Interrupt]
	Bit 5			Excessive Position Error[Interrupt]
	Bit 4			Wraparound Occurred[Interrupt]
	Bit 3			Index Pulse Observed[Interrupt]
	Bit 2			Trajectory Complete[Interrupt]
	Bit 1			Command Error[Interrupt]
	Bit 0			Busy Bit
	
	Caution:
		It should use "SelectChip" function to choose a chip before using this function.
*/	      

u8 ReadStatus(void){	
	u8 r;		
	GPIO_ResetBits(cmdP, PS);
	GPIO_ResetBits(cmdP, RD);
//	_delay_us(10);
	_delay_us(1);
	r = read_dataP();		 
	GPIO_SetBits(cmdP, RD);
	GPIO_SetBits(cmdP, PS);											
	return r;
}

// read data from 629
static u8 ReadData(void) {
	u8 r;		
	CheckBusy();
	GPIO_SetBits(cmdP, PS);	  
	GPIO_ResetBits(cmdP, RD);
//	_delay_us(10);
	_delay_us(1);
	r = read_dataP();
	GPIO_SetBits(cmdP, RD);
	GPIO_SetBits(cmdP, PS);	
	return r;
}

// write command to 629
static void WriteCommand(u8 cmd){
	CheckBusy();
	write_dataP(cmd);	
	GPIO_ResetBits(cmdP, PS);
	GPIO_ResetBits(cmdP, WR); 
	GPIO_SetBits(cmdP, WR);	
	GPIO_SetBits(cmdP, PS);	
}

/*WriteData
	Write the data to LM629.
*/
static void WriteData(u8 data){
	CheckBusy();
	write_dataP(data);		   
	GPIO_SetBits(cmdP, PS);
	GPIO_ResetBits(cmdP, WR);	
	GPIO_SetBits(cmdP, WR);	
	GPIO_SetBits(cmdP, PS);	
}

// combile four u8 into one u32
static u32 U8ToSignedLong(uc8 * data){
	return ((u32) data[0] | ((u32) data[1] << 8) | ((u32) data[2] << 16) | ((u32) data[3] << 24));
}

// Set corresponding PORT and DDR
// Reset all LM629 chip

void lm629_reset_pe(uc8 chip){
	SelectChip(chip);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~((1 << 5) | (1 << 6)));
	_delay_ms(5);
}

void lm629_abs_pos_start_done(uc8 chip, uc32 vel, sc32 pos){
//	lm629_abs_position_start(chip, vel,  pos);
	SelectChip(chip);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~(1 << 2));
	_delay_ms(2);
	lm629_abs_position_start(chip, vel,  pos);
//	while (!lm629_pos_done(chip))
//		_delay_ms(10);
}

//motion 0 straight
//motion 1 left
//motion 2 right

void lm629_abs_pos_start_done_twin(uc8 motion, uc32 vel, sc32 pos){
	if (motion==0)	{
		lm629_abs_position_start(LT_WHEEL, vel, pos*(-1));
		lm629_abs_position_start(RT_WHEEL, vel, pos);
	}
	else if (motion==1)	{
		lm629_abs_position_start(LT_WHEEL, vel, pos);
		lm629_abs_position_start(RT_WHEEL, vel, pos);
	}
	else {
		lm629_abs_position_start(LT_WHEEL, vel, pos*(-1));
		lm629_abs_position_start(RT_WHEEL, vel, pos*(-1));
	}
	SelectChip(LT_WHEEL);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~(1 << 2));
	_delay_ms(2);
	
	SelectChip(RT_WHEEL);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~(1 << 2));
	_delay_ms(2);
		
	while (!lm629_pos_done(LT_WHEEL) && !lm629_pos_done(RT_WHEEL))
		_delay_ms(10);

	lm629_stop_abruptly(LT_WHEEL);
	lm629_stop_abruptly(RT_WHEEL);
}

void lm629_init (void) {
// pins init starts
// data port io init struc is set global for fast toggling
	u8 i, status;		 
	GPIO_InitTypeDef cmdD;
	GPIO_InitTypeDef csC;
	GPIO_InitTypeDef csB;

	for(i=0; i<MAX_CHIP; i++){
		lm629_flag_ch[i]=0; 
		lm629_v[i]=0;	
		lm629_filter[i][0]=30;			 
		lm629_filter[i][1]=0;			 
		lm629_filter[i][2]=1024;			 
		lm629_filter[i][3]=2048;			 
		lm629_acc[i]=2048;	  
		lm629_pe[i]=20000;
		lm629_pos_backup[i] = 0;
	}	 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

	dataG.GPIO_Pin = D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7;   
	cmdD.GPIO_Pin = RD | WR | PS | RST; 
	csB.GPIO_Pin = CS0 | CS1; 
	csC.GPIO_Pin = CS2 | CS3; 

	dataG.GPIO_Speed = GPIO_Speed_10MHz;
	cmdD.GPIO_Speed = GPIO_Speed_10MHz;
	csC.GPIO_Speed = GPIO_Speed_10MHz;
	csB.GPIO_Speed = GPIO_Speed_10MHz;

	dataG.GPIO_Mode = GPIO_Mode_Out_PP;
	cmdD.GPIO_Mode = GPIO_Mode_Out_PP;
	csC.GPIO_Mode = GPIO_Mode_Out_PP;
	csB.GPIO_Mode = GPIO_Mode_Out_PP;

	GPIO_Init(dataP, &dataG);
	GPIO_Init(cmdP, &cmdD);
	GPIO_Init(GPIOC, &csC);
	GPIO_Init(GPIOB, &csB);
// pins init ends

	_delay_ms(80);

	GPIO_SetBits(cmdP, RD | PS | WR | RST);
	GPIO_ResetBits(GPIOB, CS0 | CS1);
	GPIO_ResetBits(GPIOC, CS2 | CS3);
	write_dataP(0x00);

    // Strobe reset, reset all chip
	GPIO_ResetBits(cmdP, RST);
	_delay_ms(1);

	GPIO_SetBits(cmdP, RST);
	_delay_ms(5);

 	// test chip availability
	for (i = 0; i < MAX_CHIP; i++){
        SelectChip(i);
        status = ReadStatus();
        if (status == 0x84 || status == 0xC4){		   //132 or 196
            WriteCommand(LM629_RSTI);			// RSTI: Reset Interrupt
            WriteData(0x00);
            WriteData(0x00);					// 0 to resets Interrupt. Reset all
		   	_delay_ms(1);
        }

        status = ReadStatus();
        if (status == 0x80 || status == 0xC0){ 			//128 or 192
            lm629_chip_available |= (1 << i);
            WriteCommand(LM629_MSKI);			// MSKI: Mask Interrupts
            WriteData(0x00);
            WriteData(0x7E);
        }
    }  
    lm629_flag = 0;
}

u8 lm629_pos_done(uc8 chip){
	if (!(lm629_chip_available & (1 << chip))) 
		return 1;
	SelectChip(chip);
	return (lm629_flag_ch[chip] & LM629_TRAJ_COMPLETION && (ReadStatus() & (1 << TrajCompleteStatus)));
}

u8 lm629_is_pe(uc8 chip){
	if (!(lm629_chip_available & (1 << chip))) 
		return 1;
	SelectChip(chip);
	return (lm629_flag_ch[chip] & LM629_POSITION_ERROR && (ReadStatus() & (1 << PositionError)));
}

u8 lm629_bp_reach(uc8 chip){
	if (!(lm629_chip_available & (1 << chip))) 
		return 1;
	SelectChip(chip);
	return (lm629_flag_ch[chip] & LM629_BREAK_POINT && (ReadStatus() & (1 << BreakPoint)));
}


void lm629_velocity_start (uc8 chip,s32 vel) {
	if (! (lm629_chip_available & (1 << chip)) ) 
		return;

	if (vel==lm629_v[chip]) {
		return;
	}
	lm629_v[chip] = vel;

	SelectChip(chip); 
		
	// If position_start is not done yet...
	if ( lm629_flag & (1 << chip) ){
		lm629_flag &= ~(1 << chip);			
		WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
		WriteData(0x00);
		// ^^^ TragCompleteStatus = 0x02
		// ^^^ reset the corr. bit
		WriteData(~(1 << TrajCompleteStatus));	// Reset Trajectory-Complete flag
	}
	
	// Load Traj.
	WriteCommand(LM629_LTRJ);	 	
	
	// ^^^ determining forward velocity or velocity	  	
	if(vel<0){
		vel = -vel;
		WriteData(0x08);
	}
	else
		WriteData(0x18);
	WriteData(0x08);
	
	WriteData((u8) (vel>>24));
	WriteData((u8) (vel>>16));
	WriteData((u8) (vel>>8));
	WriteData((u8) vel);
	
	WriteCommand(LM629_STT);	// ^^^ LM629_STT
}

// lm629_position_start
// u8 id, s32 abs velocity, s32 rel position
// 
// Trajectory control: Stop abruptly then set to position mode, load absolute velocity and relative position
//   start motion after that
// Set the trag_completion flag
void lm629_rel_position_start (uc8 chip, uc32 vel, sc32 pos) {
	if ( ! (lm629_chip_available & (1 << chip)) ) 
		return;
	
	lm629_flag |= (1 << chip);
	// ^^^ LM629_MONITOR_FLAG = 0xF0
	// ^^^ (1) clear lm629_flag_ch[x] [the lowset bits] (2) set LM629_TRAJ_COMPLETITION | COMM_ID
	lm629_flag_ch[chip] |= LM629_TRAJ_COMPLETION;

	SelectChip(chip);
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);		
	WriteData(0x00);
	WriteCommand(LM629_STT); // ^^^ STOP MOTOR
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x00);
	WriteData(0x0B);
		
	WriteData((u8) (vel>>24));
	WriteData((u8) (vel>>16));
	WriteData((u8) (vel>>8));
	WriteData((u8) vel);
		
	WriteData((u8) (pos>>24));
	WriteData((u8) (pos>>16));
	WriteData((u8) (pos>>8));
	WriteData((u8) pos);
	
	WriteCommand(LM629_STT);
}

// lm629_abs_position_start
// u8 id, s32 abs velocity, s32 abs position
// 
// Trajectory control: Stop abruptly then set to position mode, load absolute velocity and relative position
//   start motion after that
// Set the trag_completion flag
void lm629_abs_position_start (uc8 chip,uc32 vel,const s32 pos) {
	if ( ! (lm629_chip_available & (1 << chip)) ) return;
	
	lm629_flag |= (1 << chip);
	lm629_flag_ch[chip] |= LM629_TRAJ_COMPLETION;

	SelectChip(chip);
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);		
	WriteData(0x00);
	WriteCommand(LM629_STT); // ^^^ STOP MOTOR
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x00);
	WriteData(0x0A);
		
	WriteData((u8) (vel>>24));
	WriteData((u8) (vel>>16));
	WriteData((u8) (vel>>8));
	WriteData((u8) vel);
		
	WriteData((u8) (pos>>24));
	WriteData((u8) (pos>>16));
	WriteData((u8) (pos>>8));
	WriteData((u8) pos);
	
	WriteCommand(LM629_STT);
}

void lm629_read_pos_twin (s32 * pos0,s32 * pos1){
	if (! (lm629_chip_available & ((1 << RT_WHEEL) | (1 << LT_WHEEL) ) )) 
		return;
	
	SelectChip(LT_WHEEL);
	WriteCommand(LM629_RDRP);		// ^^ 0A read real position
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos0 = U8ToSignedLong(buf);
	
	SelectChip(RT_WHEEL);
	WriteCommand(LM629_RDRP);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos1 = U8ToSignedLong(buf); 
}



void lm629_read_pos_twin_home (s32 * pos0,s32 * pos1){
	if (! ((lm629_chip_available & (1 << LT_WHEEL)) && (lm629_chip_available & (1 << RT_WHEEL)))) 
		return;
			  	
	SelectChip(LT_WHEEL);
	WriteCommand(LM629_RDRP);		// ^^ 0A read real position
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos0 = U8ToSignedLong(buf);
	WriteCommand(LM629_DFH); // DFH: redefines HOME
	
	SelectChip(RT_WHEEL);
	WriteCommand(LM629_RDRP);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos1 = U8ToSignedLong(buf);

	WriteCommand(LM629_DFH); // DFH: redefines HOME	
}


void lm629_velocity_twin_start (s32 vel1, s32 vel2){
	if (! ((lm629_chip_available & (1 << LT_WHEEL)) && (lm629_chip_available & (1 << RT_WHEEL)))) 
		return;

	if (lm629_v[LT_WHEEL] != vel1) {
		lm629_v[LT_WHEEL] = vel1;

		SelectChip(LT_WHEEL);
		if ( lm629_flag & (1 << LT_WHEEL) )	{	
	
			lm629_flag &= ~(1 << LT_WHEEL);
			
			WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
			WriteData(0x00);
			// ^^^ TragCompleteStatus = 0x02
			// ^^^ reset the corr. bit
			WriteData(~(1 << TrajCompleteStatus));	// Reset Trajectory-Complete flag
		}
		WriteCommand(0x1F);	// LTRJ, Velocity
		
		if (vel1 < 0){ 
			vel1 = -vel1; 
			WriteData(0x08);
		} 
		else 
			WriteData(0x18); 
		WriteData(0x08);
		WriteData((u8) (vel1>>24));
		WriteData((u8) (vel1>>16));
		WriteData((u8) (vel1>>8));
		WriteData((u8) (vel1));
			
		SelectChip(LT_WHEEL);
		WriteCommand(LM629_STT);
	}
	
	//-------
	if (lm629_v[RT_WHEEL] != vel2) {
		lm629_v[RT_WHEEL] = vel2;
			  		
		SelectChip(RT_WHEEL);
		if ( lm629_flag & (1 << RT_WHEEL) )	{	
			lm629_flag &= ~(1 << RT_WHEEL);
			
			WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
			WriteData(0x00);
			// ^^^ TragCompleteStatus = 0x02
			// ^^^ reset the corr. bit
			WriteData(~(1 << TrajCompleteStatus));	// Reset Trajectory-Complete flag
		}
		WriteCommand(LM629_LTRJ);	// LTRJ, Velocity
	
		if (vel2 > 0)
			WriteData(0x08); 
		else { 
			vel2 = -vel2; 
			WriteData(0x18);
		} 
		WriteData(0x08);
		WriteData((u8) (vel2>>24));
		WriteData((u8) (vel2>>16));
		WriteData((u8) (vel2>>8));
		WriteData((u8) (vel2));
			
		SelectChip(RT_WHEEL);
		WriteCommand(LM629_STT);
	}
	
}


void lm629_position_twin_start (uc32 vel1,uc32 vel2, s32 pos){
	if (! ((lm629_chip_available & (1 << LT_WHEEL)) && (lm629_chip_available & (1 << RT_WHEEL)))) 
		return;
	
	lm629_flag |= (1 << LT_WHEEL) | (1 << RT_WHEEL);
	lm629_flag_ch[0] |= LM629_TRAJ_COMPLETION;
	lm629_flag_ch[1] |= LM629_TRAJ_COMPLETION;

	SelectChip(RT_WHEEL);
	WriteCommand(LM629_LTRJ);	// ^^^ LTRJ
	WriteData(0x09);
	WriteData(0x00);
	WriteCommand(LM629_STT);	// ^^^ STT ..(TURN OFF)
	
	SelectChip(LT_WHEEL);
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);
	WriteData(0x00);
	WriteCommand(LM629_STT);
	
	WriteCommand(LM629_LTRJ);	// LTRJ, Velocity + Rel Position
	WriteData(0x00);
	WriteData(0x0B);
	WriteData((u8) (vel1>>24));
	WriteData((u8) (vel1>>16));
	WriteData((u8) (vel1>>8));
	WriteData((u8) vel1);
	WriteData((u8) (pos>>24));
	WriteData((u8) (pos>>16));
	WriteData((u8) (pos>>8));
	WriteData((u8) (pos));
	
	pos = -pos;
	SelectChip(RT_WHEEL);
	WriteCommand(0x1F);	// LTRJ, Velocity + Rel Position
	WriteData(0x00);
	WriteData(0x0B);
	WriteData((u8) (vel2>>24));
	WriteData((u8) (vel2>>16));
	WriteData((u8) (vel2>>8));
	WriteData((u8) vel2);
	WriteData((u8) (pos>>24));
	WriteData((u8) (pos>>16));
	WriteData((u8) (pos>>8));
	WriteData((u8) (pos));
	
	SelectChip(LT_WHEEL);
	WriteCommand(LM629_STT);
	SelectChip(RT_WHEEL);
	WriteCommand(LM629_STT);
}

void lm629_set_pe_report (uc8 chip,uc16 pe)	{
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);			// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~(1 << PositionError));	// Reset Trajectory-Complete flag

	WriteCommand(LM629_LPEI);	// LPSI : encoder count
	WriteData((u8) (pe>>8));
	WriteData((u8) pe);

	lm629_flag_ch[chip] |= LM629_POSITION_ERROR;
}

void lm629_set_pe_stop (uc8 chip,uc16 pe)
{
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);			// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~(1 << PositionError));	// Reset Trajectory-Complete flag

	WriteCommand(LM629_LPES);	// LPSI : encoder count
	WriteData((u8) (pe>>8));
	WriteData((u8) pe);
	
	lm629_flag_ch[chip] |= LM629_POSITION_ERROR;
}


void lm629_set_rel_bp (uc8 chip,sc32 bp){
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~(1 << BreakPoint));	// Reset BreakPoint flag

	WriteCommand(LM629_SBPR);	// SBPR
	WriteData((u8) (bp>>24));
	WriteData((u8) (bp>>16));
	WriteData((u8) (bp>>8));
	WriteData((u8) bp);

	lm629_flag_ch[chip] |= LM629_BREAK_POINT;
}


void lm629_set_abs_bp (uc8 chip,sc32 bp){
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~(1 << BreakPoint));	// Reset BreakPoint flag

	WriteCommand(LM629_SBPA);	// SBPA
	WriteData((u8) (bp>>24));
	WriteData((u8) (bp>>16));
	WriteData((u8) (bp>>8));
	WriteData((u8) bp);

	lm629_flag_ch[chip] |= LM629_BREAK_POINT;
}


void lm629_zero_drive (uc8 chip){
	if ( !(lm629_chip_available & (1 << chip))) 
		return;
	
	lm629_v[chip] = 0;
	
	SelectChip(chip);
	
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);	// ^^^ zero drive + velocity mode
	WriteData(0x00);
	
	WriteCommand(LM629_STT);
}


void lm629_stop_abruptly (uc8 chip){
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;
	
	lm629_v[chip] = 0;
	
	SelectChip(chip);
	
	WriteCommand(LM629_LTRJ);
	WriteData(0x02);		// ^^^ stop abruptly
	WriteData(0x00);
	
	WriteCommand(LM629_STT);
}

void lm629_stop_smoothly(uc8 chip){
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;

	lm629_v[chip] = 0;

	SelectChip(chip);

	WriteCommand(LM629_LTRJ);
	WriteData(0x04);
	WriteData(0x00);

	WriteCommand(LM629_STT);
}



void lm629_acceleration (uc8 chip,uc32 acc) {
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;
	
	SelectChip(chip);
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x00);			// ^^^ an absoulte acc will be loaded
	WriteData(0x20);
	
	WriteData((u8) (acc>>24));
	WriteData((u8) (acc>>16));	
	WriteData((u8) (acc>>8));
	WriteData((u8) acc);
}


s32 lm629_read_pos (uc8 chip) {
	if (! (lm629_chip_available & (1 << chip))) 
		return 0;

	SelectChip(chip);
	WriteCommand(LM629_RDRP);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();

	return U8ToSignedLong(buf)+lm629_pos_backup[chip];
}

s32 lm629_read_signal (uc8 chip) {
	if (! (lm629_chip_available & (1 << chip))) 
		return 0;

	SelectChip(chip);
	WriteCommand(LM629_RDSIGS);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = 0;
	buf[0] = 0;

	return U8ToSignedLong(buf);
}

s32 lm629_read_des_pos (uc8 chip) {
	if (! (lm629_chip_available & (1 << chip))) 
		return 0;

	SelectChip(chip);
	WriteCommand(LM629_RDDP);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();

	return U8ToSignedLong(buf);
}

s32 lm629_read_pos_home (uc8 chip) {
	if( ! (lm629_chip_available & (1 << chip)) ) 
		return 0;
	
	SelectChip(chip);
	WriteCommand(LM629_RDRP);	// ^^^ Read Real Position
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	WriteCommand(LM629_DFH);	// ^^^ DFH
	return U8ToSignedLong(buf);
}

void lm629_set_filter (uc8 chip,uc16 kp,uc16 ki,uc16 kd,uc16 il) {
	if ( !(lm629_chip_available & (1 << chip)) ) 
		return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_LFIL);	// ^^^ LIFL  : Load Filter Parameters
	//Control Words
	
	WriteData(0x00);
	WriteData(0x0F);

	//kp
	WriteData((u8) kp>>8);
	WriteData((u8) kp);
	//ki
	WriteData((u8) ki>>8);
	WriteData((u8) ki);
	//kd
	WriteData((u8) kd>>8);
	WriteData((u8) kd);

	//il
	WriteData((u8) il>>8);
	WriteData((u8) il);
	
	//Update Filter	
	WriteCommand(LM629_UDF);	/// ^^^ UDF : Update Filter
}
 
void lm629_define_home (uc8 chip) {
	if( ! (lm629_chip_available & (1 << chip)) ) 
		return;
	
	SelectChip(chip);
	WriteCommand(0x02);	
}


void lm629_disable_output(void){
	u8 i;
	for(i=0; i<MAX_CHIP; i++)
		if (lm629_chip_available & (1 << i)) 
			lm629_zero_drive(i);
}

s32 lm629_read_vel(uc8 chip){
	if (!(lm629_chip_available & (1 << chip))) 
		return 0;
	SelectChip(chip);
	WriteCommand(LM629_RDRV);
	/*
	buf[3] = 0;
	buf[2] = 0;
	buf[1] = ReadData();
	buf[0] = ReadData();
	*/
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = 0;
	buf[0] = 0;
	return U8ToSignedLong(buf);
}


s32 lm629_read_des_vel(uc8 chip){
	if (!(lm629_chip_available & (1 << chip))) 
		return 0;
	SelectChip(chip);
	WriteCommand(LM629_RDDV);
	/*
	buf[3] = 0;
	buf[2] = 0;
	buf[1] = ReadData();
	buf[0] = ReadData();
	*/
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = 0;
	buf[0] = 0;
	return U8ToSignedLong(buf);
}
