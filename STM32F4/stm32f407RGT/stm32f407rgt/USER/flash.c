#include "flash.h"

uint32_t startAddress = 0x080E0000;//starting from 896KB, the beginning of last sector


void writeFlash(void)
 {


 FLASH_Unlock();// you need to unlcok flash first
 /* Clear All pending flags */
 FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR |  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

//you need to erase entire sector before write anything
 FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
 //VoltageRange_3        ((uint8_t)0x02)  /*!<Device operating range: 2.7V to 3.6V */



 FLASH_ProgramHalfWord(startAddress ,13); //halfword only for 16 bits data

FLASH_Lock();//lock flash at the end

}

//when you need to recover the maze information form flash to ram

//just simply call this function. the reading speed for flash is faster

//than write
 void readFlash(void)
 {
 u32 i, j;
 for(i=0; i<mSize; i++)
 for(j=0; j<mSize; j++)
 mazeWalls[i][j] = *(int16_t *)(startAddress + (i*mSize+j)*4);
 }
