#ifndef _XBOX_CONTROLLER_H_
#define _XBOX_CONTROLLER_H_

// We need the Windows Header and the XInput Header
// #include <windows.h>

UINT __cdecl xbox_write_thread(LPVOID app_ptr);
void terminate_thread();

#endif