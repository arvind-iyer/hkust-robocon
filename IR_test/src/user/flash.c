#include "flash.h"

const u32 start_address = 0x8076000; //starting from 400KB

void write_flash(u16 offset, s32 data)
{
    //u32 i, j;
		// First store all flash data to RAM
		const u32 data_count = FLASH_PAGE_SIZE_KB / FLASH_OFFSET_SIZE_KB;
		s32 flash_data[data_count];
		
		for (u32 i = 0; i < data_count; ++i) {
			flash_data[i] = read_flash(i);
		}
		
		flash_data[offset] = data;
		
    FLASH_Unlock();	
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(start_address);
    
    
		for (u32 i = 0; i < data_count; ++i) {
			//flash_data[i] = read_flash(i);
			FLASH_ProgramWord((u32)(start_address + i * FLASH_OFFSET_SIZE_KB), flash_data[i]);
		}

    FLASH_Lock();

}

s32 read_flash(u16 offset)
{
    return *(s32*) (start_address + offset * FLASH_OFFSET_SIZE_KB);
}
