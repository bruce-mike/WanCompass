#include <string.h>
#include <stdio.h>
#include "lpc23xx.h"
#include "gpio.h"
#include "spiflash.h"
#include "ssp.h"

static BYTE SPIWRCommand[1];

void spiFlashWriteAddress(BYTE *address);
void spiFlashWriteCommand(BYTE command, BOOL resetCS);
void spiFlashReadStatus(BYTE *status, BOOL resetCS);
BOOL spiFlashVerifyData(BYTE *data, DWORD spiReadAddress, UINT imageSize, BYTE readType);
DWORD spiFlashSwapDword( DWORD Data ); 
WORD spiFlashSwapWord(WORD Data);

void spiFlashErase(BYTE eraseType, DWORD eraseAddress)
{
	BYTE status;
	
	union
	{
		BYTE bEraseAddress[4];
		DWORD dEraseAddress;
	}SPI_Erase_Address;
	
	SPI_Erase_Address.dEraseAddress = spiFlashSwapDword(eraseAddress);
	
	//Write Enable the SPI Flash
	spiFlashWriteCommand(SPI_FLASH_WRITE_ENABLE, TRUE);	
	
	// Erase Type (Sector(4kb), Block(64kb), Chip(Entire Chip) )
	SPIWRCommand[0] = eraseType;	
	
	spiFlashWriteCommand(eraseType, FALSE);
	
  // Write Address plus Data
	spiFlashWriteAddress(SPI_Erase_Address.bEraseAddress);
	
	// Deassert CS			
	FIO0SET |= (1 << uP_SPI1_xCS);
	
	//Check Status
	spiFlashWriteCommand(SPI_FLASH_READ_STATUS, FALSE);	
	
	do
	{
		spiFlashReadStatus(&status, FALSE);		
	} while((status & SPI_FLASH_WRITE_IN_PROGRESS) == SPI_FLASH_WRITE_IN_PROGRESS);
	
	// Deassert CS			
	FIO0SET |= (1 << uP_SPI1_xCS);
}

BOOL spiFlashVerifyData(BYTE *data, DWORD spiReadAddress, UINT imageSize, BYTE readType)
{
		UINT i;
		BOOL flashVerified = TRUE;

	union
		{
			BYTE SPIRDAddress[3];
			DWORD SPIReadAddress;
		}	SPI_Read_Address;
		
		SPI_Read_Address.SPIReadAddress = spiReadAddress;
	
		//Read Type(Normal,Fast,Double)
		spiFlashWriteCommand(readType, FALSE);
	
		// Write Address to read from
		spiFlashWriteAddress(SPI_Read_Address.SPIRDAddress);
		
		for(i=0; i<imageSize; i++)
		{
			SSPReceive(data, 1);
		}
		
		// Deassert CS				
		FIO0SET |= (1 << uP_SPI1_xCS);

		return flashVerified;
}

void spiFlashWriteCommand(BYTE command, BOOL resetCS)
{
	//Write Enable the SPI Flash
	SPIWRCommand[0] = command;
	
	SSPSend(SPIWRCommand, 1); // One byte for command
	
	if(resetCS)
	{	
		// Deassert CS		
		FIO0SET |= (1 << uP_SPI1_xCS);
	}		
}

void spiFlashWriteAddress(BYTE *address)
{
	SSPSend(address,3); // SPI FLASH Address is 3 bytes
}

void spiFlashReadStatus(BYTE *status, BOOL resetCS)
{
	SSPReceive(status, 1); // One byte
	
	if(resetCS)
	{	
		// Deassert CS	
		FIO0SET |= (1 << uP_SPI1_xCS);
	}		
}

void spiFlashReadData(BYTE *data, DWORD spiReadAddress, UINT imageSize, BYTE readType)
{
	union
	{
		BYTE SPIRDAddress[4];
		DWORD SPIReadAddress;
	}	SPI_Read_Address;

	SPI_Read_Address.SPIReadAddress = spiFlashSwapDword(spiReadAddress);

	//Read Type(Normal,Fast,Double)
	spiFlashWriteCommand(readType, FALSE);

	// Write Address to read from
	spiFlashWriteAddress(SPI_Read_Address.SPIRDAddress);

	SSPReceive(data, imageSize);

	// Deassert CS			
	FIO0SET |= (1 << uP_SPI1_xCS);
}

void spiFlashWriteData(BYTE *data, DWORD startAddress, UINT imageSize)
{
	int i;
	BYTE status;

	union
	{
		BYTE bWriteAddress[4];
		DWORD dWriteAddress;
	}SPI_Write_Address;

	SPI_Write_Address.dWriteAddress = spiFlashSwapDword(startAddress);

	//Write Enable the SPI Flash
	spiFlashWriteCommand(SPI_FLASH_WRITE_ENABLE, TRUE);

	spiFlashWriteCommand(SPI_FLASH_PAGE_PROGRAM, FALSE);

	//Write Address plus Data
	spiFlashWriteAddress(SPI_Write_Address.bWriteAddress);

	for(i=0; i<imageSize; i++)
	{
		SSPSend(&data[i], 1);
	}

	// Must Deassert CS immediately after every page(256 bytes) write
	FIO0SET |= (1 << uP_SPI1_xCS);

	//Write Enable the SPI Flash
	spiFlashWriteCommand(SPI_FLASH_WRITE_ENABLE, TRUE);

	//Read SPI Flash Status	
	spiFlashWriteCommand(SPI_FLASH_READ_STATUS, FALSE);

	do
	{
		spiFlashReadStatus(&status, FALSE);

	} while(status == (SPI_FLASH_WRITE_IN_PROGRESS | SPI_FLASH_WRITE_ENABLE_LATCH));

	FIO0SET |= (1 << uP_SPI1_xCS);
}

DWORD spiFlashSwapDword(DWORD Data) 
{
	DWORD swappedData;
	
	swappedData = ((( Data & 0xFF000000 ) >> 24 ) |
					 (( Data & 0x00FF0000 ) >>  8 ) |
					 (( Data & 0x0000FF00 ) <<  8 ) |
					 (( Data & 0x000000FF ) << 24 ));
	
	return (swappedData >> 8);
}

WORD spiFlashSwapWord(WORD Data)
{	
	return((Data>>8) | (Data<<8));
}

