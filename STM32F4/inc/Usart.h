/*
 * Usart.h
 *
 *  Created on: 2014�~8��4��
 *      Author: YunKei
 */

#ifndef USART_H_
#define USART_H_
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>


class Usart{

	public:

		Usart(USART_TypeDef* UARTx, uint32_t baudrate, bool createdInstance = false);
		static Usart* getInstance(USART_TypeDef* UARTx);
		int Read(unsigned char*, int);
		void setBufferCount(int);
		int getBufferCount();
		char* getBuffer();
		static void setPrintUsart(USART_TypeDef*);
		void Print(const char*, ...);
		bool getIsDmaBusy();
		void setIsDmaBusy(bool);
		char* getRxBuffer();
		void reset();
		uint32_t getBaudrate();

	private:

		char Buffer[2048];
		char txBuffer[64];
		char rxBuffer[7];
		bool isDmaBusy;
		char* pBuffer;
		int BufferCount;
		USART_TypeDef* _Usart;
		uint32_t _baudrate;
};


#endif /* USART_H_ */
