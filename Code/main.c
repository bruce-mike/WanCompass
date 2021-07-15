#include <lpc23xx.h>

#define MAIN

#include <stdio.h>
#include "gpio.h"
#include "serial.h"
#include "serialDebug.h"
#include "timer.h"
#include "pll.h"
#include "irq.h"
#include "ssp.h"
#include "MPU9250.h"
#include "storedconfig.h"
#include "compass.h"
#include "i2c.h"
#include "software_version.h"


#define WATCHDOG_FEEDING_TIME_MS 10000 // 10 seconds
#define STAY_AWAKE_TIME_MS 500 // 0.5 seconds
#define WAIT_TIME_MS 10 // 10 mseconds

#define POR 1	

static TIMERCTL watchdogFeedingTimer;
static TIMERCTL stayAwakeTimer;

void WatchdogTimerInit(void)
{
	WDCLKSEL = 0x0; // input clock is 4MHz, divided by 4 is 1MHz
	WDTC = 30000000; // set watchdog to 30seconds, 30e6 clocks
	WDMOD = 0x03;
}

int main(void)
{
	int k; 
	int theResetSource = RSIR;
	BOOL resetInit = TRUE; // for a POR
	BOOL initDone = FALSE;
	
	RSIR = 0x000F; // reset the reset source flags
	PCONP = 0;

	// delay for JTAG to catch the processor and put it in reset	
	for(k = 0; k < 10000; k++)
	{}

	/////
	// setup the clocks
	/////
	PLL_CLOCK_SETUP();
		
	/////
	// initialize miscellaneous hardware
	/////
	hwGPIOConfig();
	
	// init vic
	init_VIC();

	/////
	// initialize the timer system
	/////
	timerInit();

	// init the spi bus
	SSPInit();
	storedConfigInit();
	I2CInitialize();

	/////
	// initialize the watchdog timer
	// timeout in 2 seconds
	/////
	initTimer(&watchdogFeedingTimer);
	startTimer(&watchdogFeedingTimer, WATCHDOG_FEEDING_TIME_MS);

	/////
	// show heartbeat for debug
	/////
	timerShowLEDHeartbeat(1000);

	/////
	// initialize serial debug port
	/////
	serialDebugInit();
		
	/////
	// initialize serial port
	/////
	serialInit();

    printf("Version[%s]  HWID[%d]\r\n", SOFTWARE_VERSION, hwGetRevNumber());

	if((theResetSource & POR) == POR) 
	{
		// then a power on reset
		printf("\nPOR\n\n");
	}
	else if((WDMOD & 0x04) == 0x04)
	{
		WDMOD = 0x03;
		// a watchdog reset
		printf("\nWDT\n\n");
		
		resetInit = FALSE;
	}
	else
	{
		// a wake on motion reset
		printf("\nINT\n\n");
		
		resetInit = FALSE;
	}
	
	WatchdogTimerInit();
	
	/////
	// feed the watchdog
	/////		
	hwPetWatchDog();
    

	while(1)
	{
		if(initDone == FALSE)
		{
			if(MPU9250() == TRUE)
			{
				initDone = TRUE;
				resetInit = TRUE;
			}
		}
		
		if(isTimerExpired(&watchdogFeedingTimer))
		{
			/////
			// feed the watchdog
			/////		
			hwPetWatchDog();

			/////
			// and restart the feeding timer
			/////
			startTimer(&watchdogFeedingTimer, WATCHDOG_FEEDING_TIME_MS);
		}
		
		serialDebugDoWork();
		serialDoWork();
		
		if(compassRun(resetInit) == TRUE)
		{
			while(1);
		}

		if(serialDebugFifoEmtpy() == TRUE && serialFifoEmtpy() == TRUE && getCompassState() == COMPASS_BOARD_STATE_IDLE && getReadContinuous() == FALSE)
		{
			initTimer(&stayAwakeTimer);
			startTimer(&stayAwakeTimer, WAIT_TIME_MS);
			while(isTimerExpired(&stayAwakeTimer) == FALSE);
			
			setSpeed(FALSE);
		}
	}
}
// move cal orientation
// low power for peripherals
// 
// MBC
// use orientation, use GPS when comes in
// compass is upside down
