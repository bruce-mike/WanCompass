
#include <lpc23xx.h>
#include <string.h>
#include "gpio.h"
#include "serialDebug.h"
#include "serial.h"
#include "timer.h"
#include "irq.h"

#define TXD0_SEL	4   		// pinsel0
#define RXD0_SEL	6				// pinsel0
#define UART0_SEL	0x01		// this value is shifted left by [T, R]xD0_SEL in pisel0 register

//
// DEBUG Port Items
//
#define DEBUG_RECEIVE_TIME_OUT 10000 // 10 seconds
#define TX_DEBUG_FIFO_LENGTH 2048
#define RX_DEBUG_FIFO_LENGTH 256

static BYTE nRxDebugAddToIndex;
static BYTE nRxDebugRemoveFromIndex;
static unsigned short nTxDebugRemoveFromIndex;
static unsigned short nTxDebugAddToIndex;
static TIMERCTL DebugReceiveTimeoutTimer;
static BYTE bDebugMessageLength = 0;
static BOOL bDebugMessageReceived = FALSE;
static BOOL bDebugTimerSet = FALSE;
static BYTE bDebugReceivedMessageLength = 0;

static BYTE rxDebugFIFO[RX_DEBUG_FIFO_LENGTH];
static BYTE bDebugReceivedMessage[RX_DEBUG_FIFO_LENGTH];
static char bDebugMessage[RX_DEBUG_FIFO_LENGTH];
static BYTE txDebugFIFO[TX_DEBUG_FIFO_LENGTH];
static BOOL serialDebugQuitFlag = FALSE;
static BOOL sendEmpty = TRUE;
static BOOL receiveEmpty = TRUE;

// functions
/////////////////////////////////////////////////////////////////////////
// RS485 on both arrow board, handheld, PCU, and CCU
/////////////////////////////////////////////////////////////////////////
 __irq void UART0_HANDLER(void)
{
	BYTE nData;

	while(U0LSR & RDR)
	{
		nData = U0RBR;
	
		if(nRxDebugAddToIndex != nRxDebugRemoveFromIndex || receiveEmpty == TRUE)
		{
			rxDebugFIFO[nRxDebugAddToIndex++] = nData;
			receiveEmpty = FALSE;
		}
	}

 VICVectAddr = 0x00;
}

// global functions
void serialDebugInitSetBaud(BOOL speed)
{
	if(speed == TRUE)
	{		
		U0LCR |= (1 << DLAB) | (EIGHTBITS << DATA_WIDTH);
		U0DLM = DLM_19200_72MHZ;
		U0DLL = DLL_19200_72MHZ;
		U0FDR = DIVADDVAL_72MHZ | (MULVAL_72MHZ << 4);
		U0LCR &= ~(1 << DLAB);	
	}
	else
	{	
		U0LCR |= (1 << DLAB) | (EIGHTBITS << DATA_WIDTH);
		U0DLM = DLM_19200_7680000HZ;
		U0DLL = DLL_19200_7680000HZ;
		U0FDR = DIVADDVAL_7680000HZ | (MULVAL_7680000HZ << 4);
		U0LCR &= ~(1 << DLAB);	
	}
}

void serialDebugInit(void)
{
	//==================
	// setup RS485 Debug port
	//==================

	// CONFIGURE THE PIN SELECTION
	// config IO pins for serial Debug
	PINSEL0 |= (UART0_SEL << TXD0_SEL) | (UART0_SEL << RXD0_SEL); 
	
	// enable the power for the serial port
	PCONP |= 1 << PCUART0;
	
	// SET UP REGS
	// SET UP UART for baud rate
	serialDebugInitSetBaud(FALSE);
	
	install_irq(UART0_INT, (void (*) (void) __irq) UART0_HANDLER, HIGHEST_PRIORITY);
	U0IER = RX_DATA_AVAILABLE_INTERRUPT_ENABLE;
}

void serialDebugDoWork(void)
{
	BYTE nData;
	
	// handle the debug data
	while(nRxDebugAddToIndex != nRxDebugRemoveFromIndex || receiveEmpty == FALSE)
	{
		if(bDebugTimerSet == TRUE && isTimerExpired(&DebugReceiveTimeoutTimer))
		{
			bDebugTimerSet = FALSE;
			// clear the input buffer
			bDebugReceivedMessageLength = 0;
			// clear the input line
			serialDebugSendData('\r');
			serialDebugSendData('\n');
		}

		U0IER = 0;
		nData = rxDebugFIFO[nRxDebugRemoveFromIndex++];
		
		if(nRxDebugRemoveFromIndex == nRxDebugAddToIndex)
		{
			receiveEmpty = TRUE;
		}
		U0IER = RX_DATA_AVAILABLE_INTERRUPT_ENABLE;
		
		if(nData == '\r')
		{
			if(bDebugReceivedMessageLength >= 4 && strncmp("quit", (char*)bDebugReceivedMessage, 4) == 0)
			{
				serialDebugQuitFlag = TRUE;
			}
			else
			{
				memcpy(bDebugMessage, bDebugReceivedMessage, bDebugReceivedMessageLength);
				bDebugMessageLength = bDebugReceivedMessageLength;
				bDebugMessageReceived = TRUE;
			}
			bDebugTimerSet = FALSE;
			
			bDebugReceivedMessageLength = 0;
			serialDebugSendData(nData);
			// for a carriage return, send aline feed
			serialDebugSendData('\n');
		}
		else if(nData >= ' ' && nData < 0x7F)
		{
			if(bDebugReceivedMessageLength == 0)
			{
				// start a timer when the first byte of a command is received
				// if the remaining data is not received within a specified amout of time
				// clear the input buffer
				bDebugTimerSet = TRUE;
				initTimer(&DebugReceiveTimeoutTimer);
				startTimer(&DebugReceiveTimeoutTimer, DEBUG_RECEIVE_TIME_OUT);
			}

			bDebugReceivedMessage[bDebugReceivedMessageLength++] = nData;
			serialDebugSendData(nData);
		}
	}
	
	while(U0LSR & THRE)
	{
		if(nTxDebugRemoveFromIndex != nTxDebugAddToIndex || sendEmpty == FALSE)
		{
			U0THR = txDebugFIFO[nTxDebugRemoveFromIndex++];
			nTxDebugRemoveFromIndex &= 0x07FF;
			if(nTxDebugRemoveFromIndex == nTxDebugAddToIndex)
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

BOOL serialDebugGetMessage(char **bMessage, BYTE *bMessageLength)
{
	BOOL returnValue = bDebugMessageReceived;
	
	if(bDebugMessageReceived == TRUE)
	{
		*bMessage = &bDebugMessage[0];
		bDebugMessage[bDebugMessageLength] = 0;
		*bMessageLength = bDebugMessageLength;
		bDebugMessageReceived = FALSE;
	}
	
	return returnValue;
}

void serialDebugSendData(BYTE nData)
{
	BYTE index;
	
	if(nTxDebugAddToIndex != nTxDebugRemoveFromIndex || sendEmpty == TRUE)
	{
		txDebugFIFO[nTxDebugAddToIndex++] = nData;
		nTxDebugAddToIndex &= 0x07FF;
		sendEmpty = FALSE;
	}

	if(nData == '\n')
	{
		for(index = 0; index < bDebugReceivedMessageLength; index++)
		{
			if(nTxDebugAddToIndex != nTxDebugRemoveFromIndex || sendEmpty == TRUE)
			{
				txDebugFIFO[nTxDebugAddToIndex++] = bDebugReceivedMessage[index];
				nTxDebugAddToIndex &= 0x07FF;
				sendEmpty = FALSE;
			}
		}
	}
}

BOOL serialDebugFifoEmtpy(void)
{
	BOOL returnValue = FALSE;
	
	if(nTxDebugRemoveFromIndex == nTxDebugAddToIndex && nRxDebugAddToIndex == nRxDebugRemoveFromIndex && bDebugMessageReceived == FALSE && bDebugReceivedMessageLength == 0)
	{
		returnValue = TRUE;
	}
	return returnValue;
}

void serialDebugSetWake(BOOL setWake)
{
	if(setWake == TRUE)
	{
		IO0_INT_EN_R &= ~(1 << GPIO_PIN_RX0);
		PINSEL0 |= (UART0_SEL << RXD0_SEL); 
		bDebugReceivedMessageLength = 0;
	}
	else
	{
		PINSEL0 &= ~(UART0_SEL << RXD0_SEL); 
		IO0_INT_EN_R |= (1 << GPIO_PIN_RX0);
	}
}

BOOL serialDebugGetQuitFlag(void)
{
	BOOL returnValue = FALSE;
	
	if(serialDebugQuitFlag == TRUE)
	{
		returnValue = TRUE;
		serialDebugQuitFlag = FALSE;
	}
	
	return returnValue;
}
