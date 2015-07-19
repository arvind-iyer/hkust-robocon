#include "flash.h"




u32 startAddress = 0x080E0000;//starting from 896KB, the beginning of last sector

void writeFlash(u8 address_num,u32 data) //address_num should start from 0 up to 255
 {

const u8 data_count=255;
	 
	 u32 flash_data[data_count];
	 
	 for (u8 i = 0; i < data_count; ++i) {
			flash_data[i] = readFlash(i);
		}
		
		flash_data[address_num] = data;
	 
	 
 FLASH_Unlock();// you need to unlcok flash first
 /* Clear All pending flags */
 FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR |  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

//you need to erase entire sector before write anything
 FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
 //VoltageRange_3        ((uint8_t)0x02)  /*!<Device operating range: 2.7V to 3.6V */

		for (u8 i = 0; i < data_count; ++i) {   //write back the whole sector memory
	
			FLASH_ProgramWord((u32)(startAddress + i * 4), flash_data[i]);
		}

 FLASH_Lock();//lock flash at the end

}

//when you need to recover the maze information form flash to ram

//just simply call this function. the reading speed for flash is faster

//than write

u32 readFlash(u16 address_num)
 {
 return *(int16_t *)(startAddress+address_num*4);
	
 }

 
// #include "flash.h"

//const u32 start_address = 0x8076000; //starting from 400KB

//void write_flash(u16 offset, s32 data)
//{
//    //u32 i, j;
//		// First store all flash data to RAM
//		const u32 data_count = FLASH_PAGE_SIZE_KB / FLASH_OFFSET_SIZE_KB;
//		s32 flash_data[data_count];
//		
//		for (u32 i = 0; i < data_count; ++i) {
//			flash_data[i] = read_flash(i);
//		}
//		
//		flash_data[offset] = data;
//		
//    FLASH_Unlock();	
//    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
//    FLASH_ErasePage(start_address);
//    
//    
//		for (u32 i = 0; i < data_count; ++i) {
//			//flash_data[i] = read_flash(i);
//			FLASH_ProgramWord((u32)(start_address + i * FLASH_OFFSET_SIZE_KB), flash_data[i]);
//		}

//    FLASH_Lock();

//}

//s32 read_flash(u16 offset)
//{
//    return *(s32*) (start_address + offset * FLASH_OFFSET_SIZE_KB);
//}
