/*****************************************************************************
 *   i2c.h:  Header file for NXP LPC23xx/24xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.07.19  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/

#ifndef __I2C_H 
#define __I2C_H

// WARNING: MAY COLLIDE WITH COMMAND CHAR ARRAY DATA!!!
#define END 0xFE  		// WARNING: MAY COLLIDE WITH COMMAND CHAR ARRAY DATA!!!
// WARNING: MAY COLLIDE WITH COMMAND CHAR ARRAY DATA!!! 



#define BUFSIZE			16
#define MAX_TIMEOUT	0x0FFF //how many cycles??

#define I2CMASTER		0x01
#define I2CSLAVE		0x02

//************************
//************************
//************************
//9634 defs
//************************
//************************
// BASE ADDRESSES
#define PCA9634_SYS_LEDS_AND_PWM_WRITE  0xE6 // SUBADR2 register: E4h or 1110 010X
#define PCA9634_SYS_LEDS_AND_PWM_READ 	0xE7

// REGISTER DEFS
#define MODE1				0x00		
#define AUTO_MODE1			0x80		// auto-inc bit set
#define MODE2				0x01
#define AUTO_MODE2			0x81		// auto-inc bit set
#define PWM0				0x02	
#define AUTO_PWM0			0x82		// auto-inc bit set	
#define PWM1				0x03
#define PWM2				0x04
#define PWM3				0x05
#define PWM4				0x06
#define PWM5				0x07
#define PWM6				0x08
#define PWM7				0x09
#define GRPPWM				0x0A
#define AUTO_GRPPWM			0x8A		// auto-inc bit set
#define GRPFREQ				0x0B
#define LEDOUT0				0x0C
#define AUTO_LEDOUT0		0x8C		// auto-inc bit set
#define LEDOUT1				0x0D
#define SUBADR1				0x0E
#define SUBADR2				0x0F
#define SUBADR3				0x10
#define ALLCALLADR			0x11
#define MODE1_SLEEP			0x00 // 0 = normal, 0x10 = sleep
#define MODE1_ALLCALL		0x01 // enable

#define MODE2_DMBLNK		0x00 	// 0x20 for Group control = blinking, 0 for  Group control = dimming
#define MODE2_INVRT			0x10	// 1 for  output logic state inverted; value to use when external driver used; applicable when OE = 0
									// 0 for output logic state not inverted; value to use when no external driver used; applicable when OE = 0
#define MODE2_OCH			0x08	// 1 for outputs change on ACK, 0 for outputs change on STOP command[2]
#define MODE2_OUTDRV		0x04	// 1 for  the 8 LED outputs are configured with a totem-pole structure, 0 for  the 8 LED outputs are configured with an open-drain structure
#define MODE2_OUTNE		0x01	// 3 for reserved, 2 for when OE = 1 (output drivers not enabled), LEDn = high-impedance,
									// 1 for when OE = 1 (output drivers not enabled):
									// 			LEDn = 1 when OUTDRV = 1
									// 			LEDn = high-impedance when OUTDRV = 0 (same as OUTNE[1:0] = 10)
									// 0 for  when OE = 1 (output drivers not enabled), LEDn = 0

#define PWMn_DATA	0xFF			// IDC7[7:0] - PWMn Individual Duty Cycle
									// A 97 kHz fixed frequency signal is used for each output. Duty cycle is controlled through
									// 256 linear steps from 00h (0 % duty cycle = LED output off) to FFh
									// (99.6 % duty cycle = LED output at maximum brightness). Applicable to LED outputs
									// programmed with LDRx = 10 or 11 (LEDOUT0 and LEDOUT1 registers).
									// duty cycle = IDC[7:0] / 256
#define GRPPWM_DATA	0xFF			// GDC[7:0]
									// When DMBLNK bit (MODE2 register) is programmed with 0, a 190 Hz fixed frequency
									// signal is superimposed with the 97 kHz individual brightness control signal. GRPPWM is
									// then used as a global brightness control allowing the LED outputs to be dimmed with the
									// same value. The value in GRPFREQ is then a ‘Don’t care’.
									// General brightness for the 8 outputs is controlled through 256 linear steps from 00h
									// (0 % duty cycle = LED output off) to FFh (99.6 % duty cycle = maximum brightness).
									// Applicable to LED outputs programmed with LDRx = 11 (LEDOUT0 and LEDOUT1
									// registers).
									// When DMBLNK bit is programmed with logic 1, GRPPWM and GRPFREQ registers
									// define a global blinking pattern, where GRPFREQ contains the blinking period (from
									// 24 Hz to 10.73 s) and GRPPWM the duty cycle (ON/OFF ratio in %).
									// duty cycle = GDC[7:0] / 256
#define GRPFREQ_DATA	0xFF		// GFRQ[7:0]
									// GRPFREQ is used to program the global blinking period when DMBLNK bit (MODE2
									// register) is equal to 1. Value in this register is a ‘Don’t care’ when DMBLNK = 0.
									// Applicable to LED outputs programmed with LDRx = 11 (LEDOUT0 and LEDOUT1
									// registers).
									// Blinking period is controlled through 256 linear steps from 00h (41 ms, frequency 24 Hz)
									// to FFh (10.73 s).
									// global blinking period = GFRQ[7:0] + 1 / 24 (in seconds)
									
									// 0Ch LEDOUT0 7:6 LDR3 R/W 00* LED3 output state control
									//			 5:4 LDR2 R/W 00* LED2 output state control
									//			 3:2 LDR1 R/W 00* LED1 output state control
									//			 1:0 LDR0 R/W 00* LED0 output state control
									// 0Dh LEDOUT1 7:6 LDR7 R/W 00* LED7 output state control
									//			 5:4 LDR6 R/W 00* LED6 output state control
									//			 3:2 LDR5 R/W 00* LED5 output state control
									//			 1:0 LDR4 R/W 00* LED4 output state control
									// LDRx = 00 — LED driver x is off (default power-up state).
									// LDRx = 01 — LED driver x is fully on (individual brightness and group dimming/blinking
									// not controlled).
									// LDRx = 10 — LED driver x individual brightness can be controlled through its PWMx
									// register.
									// LDRx = 11 — LED driver x individual brightness and group dimming/blinking can be
									// controlled through its PWMx register and the GRPPWM registers.
									
									// The active LOW output enable (OE) pin, allows to enable or disable all the LED outputs at
									// the same time.
									// • When a LOW level is applied to OE pin, all the LED outputs are enabled and follow
									// the output state defined in the LEDOUT register with the polarity defined by INVRT bit
									// (MODE2 register).
									// • When a HIGH level is applied to OE pin, all the LED outputs are programmed to the
									// value that is defined by OUTNE[1:0] in the MODE2 register.
									
									// The OE pin can be used as a synchronization signal to switch on/off several PCA9634
									// devices at the same time. This requires an external clock reference that provides blinking
									// period and the duty cycle.
									// The OE pin can also be used as an external dimming control signal. The frequency of the
									// external clock must be high enough not to be seen by the human eye, and the duty cycle
									// value determines the brightness of the LEDs.
									// Remark: Do not use OE as an external blinking control signal when internal global
									// blinking is selected (DMBLNK = 1, MODE2 register) since it will result in an undefined
									// blinking pattern. Do not use OE as an external dimming control signal when internal global
									// dimming is selected (DMBLNK = 0, MODE2 register) since it will result in an undefined
									// dimming pattern.
									//OUTNE1	OUTNE0	LED	outputs
									//0		0		0
									//0 		1 		1 if OUTDRV = 1, high-impedance if OUTDRV = 0
									//1 		0 		high-impedance
									//1 		1 		reserved
									
									// H BRIDGE
									//Table 11. Truth table in normal operating conditions
									//INA	INB	DIAGA/ENA	DIAGB/ENB	OUTA	OUTB	Operating mode
									//1		1	1			1			H		H		Brake to VCC
									//1		0	1			1			H		L		Clockwise (CW)
									//0		1	1			1			L		H		Counterclockwise (CCW)
									//0		0	1			1			L		L		Brake to GND				

// LED output control
// LED 0 - ACT_PWM
// LED 1 - ACT_A
// LED 2 - ACT_B
// LED 3 - ACT_ENA - active low to enable H-Bridge, high to disable the bridge
// LED 4 - xA_OK - low to turn on LED X6
// LED 5 - xB_OK - low to turn on LED X8
// LED 6 - unused
// LED 7 - unused

//************************
//************************
//************************
//MISC I2C DEFS
//************************
//************************

#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */
#define I2CONSET_AA			0x00000004
#define I2CONSET_SI			0x00000008
#define I2CONSET_STO		0x00000010
#define I2CONSET_STA		0x00000020

#define I2CONCLR_AAC		0x00000004  /* I2C Control clear Register */
#define I2CONCLR_SIC		0x00000008
#define I2CONCLR_STAC		0x00000020
#define I2CONCLR_I2ENC	0x00000040

#define I2DAT_I2C				0x00000000  /* I2C Data Reg */
#define I2ADR_I2C				0x00000000  /* I2C Slave Address Reg */
//#define I2SCLH_SCLH			0x00000080  /* I2C SCL Duty Cycle High Reg */
//#define I2SCLL_SCLL			0x00000080  /* I2C SCL Duty Cycle Low Reg */



#define I2SCLH_SCLH			90  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL			90  /* I2C SCL Duty Cycle Low Reg */
#define I2SCLH_SCLH_SLOW			10  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL_SLOW			10  /* I2C SCL Duty Cycle Low Reg */


#define PCLK_I2C0       14					/* peripheral clock select bit location WPN */



//extern unsigned long I2CInit          (unsigned long I2cMode );

#define I2C_HANGUP_TIMER_MS 100

//======================================================




//=============================================================================

typedef enum eI2CTransferState
{
	eI2C_TRANSFER_STATE_IDLE,
	eI2C_TRANSFER_STATE_PENDING,
	eI2C_TRANSFER_STATE_TRANSFERRING,
	eI2C_TRANSFER_STATE_COMPLETE,
	eI2C_TRANSFER_STATE_FAILED
}eI2C_TRANSFER_STATE;

void I2CInitialize(void);
BOOL i2cWriteByte(BYTE address, BYTE subAddress, BYTE data);
BOOL i2cWriteBytes(BYTE address, BYTE subAddress, BYTE count, BYTE *data);
BOOL i2cReadByte(BYTE address, BYTE subAddress, BYTE *dest);
BOOL i2cReadBytes(BYTE address, BYTE subAddress, BYTE count, BYTE *dest);
BOOL i2cDataTransaction(BYTE *writeData, int nWriteLength, int nReadLength, BYTE nSLA_W, BYTE subAddress, BYTE *theData);

void I2CSetBaud(BOOL speed);
//=======================================================
#endif /* end __I2C_H */
/****************************************************************************
**                            End Of File
*****************************************************************************/
