
#include <lpc23xx.h>
#include <string.h>
#include "gpio.h"
#include "serial.h"
#include "serialDebug.h"
#include "timer.h"
#include "irq.h"

#define TXD3_SEL	0 //24	// pinsel0
#define RXD3_SEL	2 //26	// pinsel0
#define UART3_SEL	0x02		// ...and so forth

//
// Port Items
//
#define RECEIVE_TIME_OUT 10000 // 10 seconds
#define TX_FIFO_LENGTH 2048
#define RX_FIFO_LENGTH 256

static BYTE nRxAddToIndex;
static BYTE nRxRemoveFromIndex;
static unsigned short nTxRemoveFromIndex;
static unsigned short nTxAddToIndex;
static TIMERCTL ReceiveTimeoutTimer;
static BYTE bSMessageLength = 0;
static BOOL bMessageReceived = FALSE;
static BOOL bTimerSet = FALSE;
static BYTE bReceivedMessageLength = 0;

static BYTE rxFIFO[RX_FIFO_LENGTH];
static BYTE bReceivedMessage[RX_FIFO_LENGTH];
static char bSMessage[RX_FIFO_LENGTH];
static BYTE txFIFO[TX_FIFO_LENGTH];
static BOOL serialQuitFlag = FALSE;
static BOOL sendEmpty = TRUE;
static BOOL receiveEmpty = TRUE;

// functions
/////////////////////////////////////////////////////////////////////////
// RS485 on both arrow board, handheld, PCU, and CCU
/////////////////////////////////////////////////////////////////////////
 __irq void UART3_HANDLER(void)
{
	BYTE nData;

	while(U3LSR & RDR)
	{
		nData = U3RBR;

		if(nRxAddToIndex != nRxRemoveFromIndex || receiveEmpty == TRUE)
		{
			rxFIFO[nRxAddToIndex++] = nData;
			receiveEmpty = FALSE;
		}
	}

 VICVectAddr = 0x00;
}

// global functions
void serialInitSetBaud(BOOL speed)
{
	if(speed == TRUE)
	{		
		U3LCR |= (1 << DLAB) | (EIGHTBITS << DATA_WIDTH);
		U3DLM = DLM_19200_72MHZ;
		U3DLL = DLL_19200_72MHZ;
		U3FDR = DIVADDVAL_72MHZ | (MULVAL_72MHZ << 4);
		U3LCR &= ~(1 << DLAB);	
	}
	else
	{	
		U3LCR |= (1 << DLAB) | (EIGHTBITS << DATA_WIDTH);
		U3DLM = DLM_19200_7680000HZ;
		U3DLL = DLL_19200_7680000HZ;
		U3FDR = DIVADDVAL_7680000HZ | (MULVAL_7680000HZ << 4);
		U3LCR &= ~(1 << DLAB);	
	}
}

void serialInit(void)
{
	//==================
	// setup RS485 port
	//==================
	
	// config IO pins for serial
	PINSEL0 |= (UART3_SEL << TXD3_SEL) | (UART3_SEL << RXD3_SEL);
	
	// enable the power for the serial port
	PCONP |= 1 << PCUART3;
	
	// SET UP REGS
	// SET UP UART for baud rate
	serialInitSetBaud(FALSE);
	
	install_irq( UART3_INT, (void (*) (void) __irq) UART3_HANDLER, HIGHEST_PRIORITY );
	U3IER = RX_DATA_AVAILABLE_INTERRUPT_ENABLE;
}

void serialDoWork(void)
{
	BYTE nData;
	
	// handle the data
	while(nRxAddToIndex != nRxRemoveFromIndex || receiveEmpty == FALSE)
	{
		if(bTimerSet == TRUE && isTimerExpired(&ReceiveTimeoutTimer))
		{
			bTimerSet = FALSE;
			// clear the input buffer
			bReceivedMessageLength = 0;
			// clear the input line
		}

		U3IER = 0;
		nData = rxFIFO[nRxRemoveFromIndex++];
		
		if(nRxRemoveFromIndex == nRxAddToIndex)
		{
			receiveEmpty = TRUE;
		}
		U3IER = RX_DATA_AVAILABLE_INTERRUPT_ENABLE;
		
		if(nData == '\r')
		{
			if(bReceivedMessageLength >= 4 && strncmp("quit", (char*)bReceivedMessage, 4) == 0)
			{
				serialQuitFlag = TRUE;
			}
			else
			{
				memcpy(bSMessage, bReceivedMessage, bReceivedMessageLength);
				bSMessageLength = bReceivedMessageLength;
				bMessageReceived = TRUE;
			}
			bTimerSet = FALSE;
			
			bReceivedMessageLength = 0;
		}
		else if(nData >= ' ' && nData < 0x7F)
		{
			if(bReceivedMessageLength == 0)
			{
				// start a timer when the first byte of a command is received
				// if the remaining data is not received within a specified amout of time
				// clear the input buffer
				bTimerSet = TRUE;
				initTimer(&ReceiveTimeoutTimer);
				startTimer(&ReceiveTimeoutTimer, RECEIVE_TIME_OUT);
			}

			bReceivedMessage[bReceivedMessageLength++] = nData;
		}
	}
	
	while(U3LSR & THRE)
	{
		if(nTxRemoveFromIndex != nTxAddToIndex || sendEmpty == FALSE)
		{
			U3THR = txFIFO[nTxRemoveFromIndex++];
			nTxRemoveFromIndex &= 0x07FF;
			if(nTxRemoveFromIndex == nTxAddToIndex)
			{
				sendEmpty = TRUE;
			}
		}
		else
		{
			break;
		}
	}
}

BOOL serialGetMessage(char **bMessage, BYTE *bMessageLength)
{
	BOOL returnValue = bMessageReceived;
	
	if(serialDebugGetMessage(bMessage, bMessageLength) == TRUE)
	{
		returnValue = TRUE;
	}
	else if(bMessageReceived == TRUE)
	{
		*bMessage = &bSMessage[0];
		bSMessage[bSMessageLength] = 0;
		*bMessageLength = bSMessageLength;
		bMessageReceived = FALSE;
	}
	
	return returnValue;
}

void serialSendData(BYTE nData)
{
	if(nTxAddToIndex != nTxRemoveFromIndex || sendEmpty == TRUE)
	{
		txFIFO[nTxAddToIndex++] = nData;
		nTxAddToIndex &= 0x07FF;
		sendEmpty = FALSE;
	}
}

BOOL serialFifoEmtpy(void)
{
	BOOL returnValue = FALSE;
	
	if(nTxRemoveFromIndex == nTxAddToIndex && nRxAddToIndex == nRxRemoveFromIndex && bReceivedMessageLength == FALSE && bReceivedMessageLength == 0)
	{
		returnValue = TRUE;
	}
	return returnValue;
}

void serialSetWake(BOOL setWake)
{
	if(setWake == TRUE)
	{
		IO0_INT_EN_R &= ~(1 << GPIO_PIN_RX3);
		PINSEL0 |= (UART3_SEL << RXD3_SEL); 
		bReceivedMessageLength = 0;
	}
	else
	{
		PINSEL0 &= ~(UART3_SEL << RXD3_SEL); 
		IO0_INT_EN_R |= (1 << GPIO_PIN_RX3);
	}
}

BOOL serialGetQuitFlag(void)
{
	BOOL returnValue = FALSE;
	
	if(serialQuitFlag == TRUE)
	{
		returnValue = TRUE;
		serialQuitFlag = FALSE;
	}
	
	return returnValue;
}
