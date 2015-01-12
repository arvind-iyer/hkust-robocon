extern "C" __declspec(dllexport) unsigned int __stdcall Init_can(unsigned char com_NUM,unsigned char Model,unsigned int CanBaudRate,unsigned char SET_ID_TYPE,unsigned char FILTER_MODE,unsigned char RXF[],unsigned char RXM[]);
extern "C" __declspec(dllexport) unsigned int __stdcall Quit_can();
extern "C" __declspec(dllexport) unsigned int __stdcall Can_send(unsigned char IDbuff[],unsigned char Databuff[],unsigned char FreamType,unsigned char Bytes);
extern "C" __declspec(dllexport) unsigned int __stdcall Can_receive(unsigned char IDbuff[],unsigned char Databuff[],unsigned char *FreamType,unsigned char *Bytes);



