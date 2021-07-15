/******************************************************************************/
/* RETARGET.C: 'Retarget' layer for target-dependent low level functions      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <lpc23xx.h>
#include <stdio.h>
#include "gpio.h"
#include "serial.h"
#include "serialDebug.h"
#include "timer.h"

#pragma import(__use_no_semihosting_swi)


extern int  sendchar(int ch);  /* in serial.c */

/*----------------------------------------------------------------------------
 *       sendchar:  Write a character to Serial Port attached to UART0
 *                  called by printf through retarget.c
 *---------------------------------------------------------------------------*/
int sendchar (int ch)
{
	int nRetVal = 0;

	if(ch != '\n' && ch != '\r' && (ch < ' ' || ch > 0x7E))
	{
		return nRetVal;
	}

	if(ch == '\n')
	{
		// for a line feed, send a carriage return
		serialSendData('\r');
		serialDebugSendData('\r');
	}
	
	serialSendData(ch);
	serialDebugSendData(ch);
	return nRetVal;
}

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;


int fputc(int ch, FILE *f) {
  return (sendchar(ch));
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int ch) {
  sendchar(ch);
}


void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}
