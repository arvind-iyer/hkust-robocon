#include "flash.h"




u32 startAddress = 0x080E0000;//starting from 896KB, the beginning of last sector

void writeFlash(u8 address_num,u16 data) //address_num should start from 0
 {

 FLASH_Unlock();// you need to unlcok flash first
 /* Clear All pending flags */
 FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR |  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

//you need to erase entire sector before write anything
 FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
 //VoltageRange_3        ((uint8_t)0x02)  /*!<Device operating range: 2.7V to 3.6V */


 FLASH_ProgramHalfWord(startAddress+address_num*4 ,data); //halfword only for 16 bits data

 FLASH_Lock();//lock flash at the end

}

//when you need to recover the maze information form flash to ram

//just simply call this function. the reading speed for flash is faster

//than write

u16 readFlash()
 {
u16 read_data = *(int16_t *)(startAddress+4);
	 return read_data;
 }
