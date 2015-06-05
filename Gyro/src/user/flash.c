#include "flash.h"



uint32_t start_address = 0x8019000; //starting from 100KB

void write_flash(u16 offset, s32 data)
{
    u32 i, j;
    FLASH_Unlock();	
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(start_address);
    
    FLASH_ProgramWord((u32)(start_address + offset), (u32) data);

    FLASH_Lock();

}

s32 read_flash(u16 offset)
{
    return *(s32*) (start_address+offset);
}


