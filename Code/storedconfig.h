///////////////////////////////////////////////////////
// stored configuration data
///////////////////////////////////////////////////////
#ifndef STOREDCONFIG_H
#define STOREDCONFIG_H

#define MAX_TP_CALIBRATION_COEFFICIENTS 7

typedef struct storedInfo
{
	unsigned int nStructFlags;
	sCALIBRATION_ACAL calibrationDataACal;
	sCALIBRATION_MCAL calibrationDataMCal;
	unsigned char padding[256 - (sizeof(unsigned int) + sizeof(sCALIBRATION_ACAL) + sizeof(sCALIBRATION_MCAL))];
}STORED_INFO;

//===================================
// initialization
//===================================
STORED_INFO* storedConfigInit(void);

//===================================
// get routines
//===================================
sCALIBRATION_ACAL *storedConfigGetACAL(void);
sCALIBRATION_MCAL *storedConfigGetMCAL(void);

//===================================
// set routines
//===================================
void storedConfigSetACAL(sCALIBRATION_ACAL *theACAL);
void storedConfigSetMCAL(sCALIBRATION_MCAL *theMCAL);
void storedConfigEraseChip(void);

#endif		// STOREDCONFIG_H
