/*****************************************************************************
 *   ssp.c:  SSP C file for NXP LPC23xx/24xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.07.20  ver 1.00    Prelimnary version, first Release
 *
*****************************************************************************/
#include "LPC23xx.h"			/* LPC23XX/24xx Peripheral Registers */
#include <stdio.h>
#include "gpio.h"
#include "ssp.h"
#include "irq.h"

#define uP_SPI1_SCK		7		// SPI1 Flash SClock
#define uP_SPI1_MISO 	8		// SPI1 Flash MISO
#define uP_SPI1_MOSI	9		// SPI1 Flash MOSI
#define uP_SPI1_HOLD 28
#define uP_SPI1_WP   29

/*****************************************************************************
** Function name:		SSPInit
**
** Descriptions:		SSP port initialization routine
**				
** parameters:			None
** Returned value:		true or false, if the interrupt handler
**										can't be installed correctly, return false.
** 
*****************************************************************************/
DWORD SSPInit(void)
{
	BYTE i, Dummy=Dummy;

	/* Configure PIN connect block SSP1 port SCK1, MISO1, MOSI1, and SSEL1 */		
	/* NOTE: ALWAYS SET SSEL1 OF PINSEL0 AS A GPIO PIN. THIS ALLOWS SOFTWARE CONTROL OF 
	THE SPI CHIP-SELECT PIN */	
	PINSEL0 |= 0x000A8000;
	FIO0SET |= (1 << uP_SPI1_xCS);	
						
	// set directions
	FIO0DIR |=	(OUT << uP_SPI1_xCS ) |
						(OUT << uP_SPI1_SCK ) |	
						(IN  << uP_SPI1_MISO) |
						(OUT << uP_SPI1_MOSI);
						
	FIO0CLR =	(1 << uP_SPI1_xCS ) |
						(1 << uP_SPI1_SCK ) |	
						(1 << uP_SPI1_MOSI) ;
	
	FIO0SET |= (1 << uP_SPI1_xCS);	
						
	//**************
	//******* PORT 4
	//**************
	// the only port4 pins are SPI1.
	PINSEL9 = 0x00000000;
	
	FIO4DIR = (OUT << uP_SPI1_HOLD) |
						(OUT << uP_SPI1_WP  );

	FIO4SET = (1 << uP_SPI1_HOLD) |
						(1 << uP_SPI1_WP  );
						
	// enable the device power
	PCONP |= 1 << PCSSP1;

	/* Set DSS data to 8-bit, Frame format SPI, CPOL = 0, CPHA = 0, and SCR is 15 */
	SSP1CR0 = 0x0707;

	/* SSP1CPSR clock prescale register, master mode, minimum divisor is 0x02 */
	SSP1CPSR = 0x2;

	for ( i = 0; i < FIFOSIZE; i++ )
	{
		Dummy = SSP1DR;		/* clear the RxFIFO */
	}	

	/* Device select as master, SSP Enabled */
	SSP1CR1 = SSPCR1_SSE;

	return( TRUE );
}

/*****************************************************************************
** Function name:		SSPSend
**
** Descriptions:		Send a block of data to the SSP port, the 
**									first parameter is the buffer pointer, the 2nd 
**									parameter is the block length, and the 3rd parameter is 
**									platform type.
**
** parameters:			buffer pointer, block length, and ePlatformType
** Returned value:	None
** 
*****************************************************************************/
void SSPSend(BYTE *buf, DWORD Length)
{
	DWORD i;
	BYTE Dummy = Dummy;

	//Assert CS
	FIO0CLR |= (1 << uP_SPI1_xCS);	

	for ( i = 0; i < Length; i++ )
	{
		/* Move on only if NOT busy and TX FIFO not full. */
		while((SSP1SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF);

		SSP1DR = buf[i];

		while((SSP1SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE);

		/* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO 
		on MISO. Otherwise, when SSP1Receive() is called, previous data byte
		is left in the FIFO. */
		Dummy = SSP1DR;
	}

	return; 
}

/*****************************************************************************
** Function name:		SSPReceive
** Descriptions:		the module will receive a block of data from 
**									the SSP, the 2nd parameter is the block 
**									length, and the 3rd parameter is platform type.
** parameters:			buffer pointer, block length, ePlatformType
** Returned value:	None
** 
*****************************************************************************/
void SSPReceive( BYTE *buf, WORD Length )
{
	WORD i;

	for ( i = 0; i < Length; i++ )
	{
		/* As long as Receive FIFO is not empty, I can always receive. */
		/* if it's a peer-to-peer communication, SSPDR needs to be written
		before a read can take place. */
		SSP1DR = 0xFF;

		/* Wait until the Busy bit is cleared */
		while((SSP1SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE);

		buf[i] = SSP1DR;
	}

	return; 
}
