// support functions particlar to the driver board
// GPIO config, control of periphs and status bits
// connected straight to uP.

// UART/I2C/ADC etc handled in respective .c and .h files

#include <stdio.h>




#include <lpc23xx.h>
#include "gpio.h"
#include "serial.h"
#include "timer.h"
#include "pll.h"
#include "irq.h"

#define THE_DIRECTION0 FIO0DIR
#define THE_DIRECTION1 FIO1DIR
#define THE_DIRECTION2 FIO2DIR

#define PIN_MAPPING_ASSIGNMENT(THE_PIN, THE_PIN_NUMBER, THE_PORT_NUMBER, THE_PIN_DIRECTION, THE_INITIAL_VALUE) \
		GPIO_PIN_##THE_PIN = THE_PIN_NUMBER; \
		GPIO_PORT_##THE_PIN = THE_PORT_NUMBER; \
		THE_DIRECTION##THE_PORT_NUMBER |= (THE_PIN_DIRECTION  << GPIO_PIN_##THE_PIN); \
		FIO##THE_PORT_NUMBER##THE_INITIAL_VALUE |= (1  << GPIO_PIN_##THE_PIN);

// GPIO CONFIG
// set up I/Os as in/out, set default states
// peripheral pins (UART/ADC/SPI/ETC) are *NOT* configured here.

// NB: LAST STEP IS TO DISABLE ETM IN SW
// it is configured on boot by sampling RTCK.
// The value written to the config reg is xRTCK.  
// As drawn, this enables ETM wand hoses some PORT2 ops

// *********************
// *********************
// *********************
// DISABLE ETM (embedded trace - dorks up PORT2 pins 9:0)
static void hwDisableETM(void) 
{
	
	//FIXME MAGIC NUMBER
	PINSEL10 = 0x00000000;
}

void hwGPIOConfig(void)
{
	SCS |= 0x1;	
	// SETUP PINSEL, DIRECTION, AND INITIAL VALUES
	PINSEL0 = 0x00000000; 
	PINSEL1 = 0x00000000;
	PINSEL2 = 0x00000000;
	PINSEL3 = 0x00000000;	
	PINSEL6 = 0x00000000;		
	PINSEL7 = 0x00000000;
	PINSEL9 = 0x00000000;
	
	FIO0DIR = 0;
	FIO1DIR = 0;
	FIO2DIR = 0;

	// ASSIGN THE PINS
	PIN_MAPPING_ASSIGNMENT( REV0, 0, 1, IN, CLR)
	PIN_MAPPING_ASSIGNMENT( REV1, 1, 1, IN, CLR)
	PIN_MAPPING_ASSIGNMENT( REV2, 4, 1, IN, CLR)
	PIN_MAPPING_ASSIGNMENT( uP_AUX_LED, 15, 1, OUT, CLR)
						
	// the last step - disable ETM
	hwDisableETM();
}

void hwPetWatchDog(void)
{
	WDFEED = 0xAA;
	WDFEED = 0x55;
}

//=====================
// Read CCU Row Number
//=====================
BYTE hwGetRevNumber(void)
{
	unsigned long port;	

	// model select on port0
	port = FIO1PIN;	

	// mask off Revision bits
	port = ((port & (1 << GPIO_PIN_REV0)) >> GPIO_PIN_REV0) | (((port & (1 << GPIO_PIN_REV1)) >> GPIO_PIN_REV1) << 1)  | (((port & (1 << GPIO_PIN_REV2)) >> GPIO_PIN_REV2) << 2);

	return port;
}
