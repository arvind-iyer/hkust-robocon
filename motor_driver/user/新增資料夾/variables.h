extern unsigned char		uint8[4];
extern unsigned short int	uint16[4];
extern signed char			int8[4];
extern signed short int		int16[4];
extern float				float32[4];

void uart_send_byte	(unsigned char data);

void send_uint8		(char index);
void send_uint16	(char index);
void send_int8		(char index);
void send_int16		(char index);
void send_float32	(char index);

void set_uint8		(char index,	unsigned char data);
void set_uint16		(char index,	unsigned short int data);
void set_int8		(char index,	signed char data);
void set_int16		(char index,	signed short int data);
void set_float32	(char index,	float data);

void set_and_send_uint8		(char index,	unsigned char data);
void set_and_send_uint16	(char index,	unsigned short int data);
void set_and_send_int8		(char index,	signed char data);
void set_and_send_int16		(char index,	signed short int data);
void set_and_send_float32	(char index,	float data);
