#include <lpc23xx.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gpio.h"
#include "irq.h"
#include "timer.h"
#include "i2c.h"
#include "MPU9250.h"
#include "compass.h"
#include "serial.h"
#include "serialDebug.h"

#define TIME_DELAY(X) initTimer(&delayTimer); startTimer(&delayTimer, X); while(isTimerExpired(&delayTimer) == FALSE);
#define BANK(theBank) writeByte(MPU9250_ADDRESS, BANK_SELECT, theBank << 4);

static short tempCount; // Temperature raw count output
static short magCount[3]; // Stores the 16-bit signed magnetometer sensor output
static short accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
static unsigned long lastUpdate = 0; // used to calculate integration interval
static unsigned long Now = 0;        // used to calculate integration interval
static TIMERCTL delayTimer;
// Scale resolutions per LSB for the sensors
static float aRes, mRes;
static ASCALE Ascale = AFS_2G;
static MSCALE Mscale = MFS_16BITS;
static BYTE AccelerometerFilter = 0x06;

// Variables to hold latest sensor data values
float ax, ay, az, mx, my, mz;
// Factory mag calibration and mag bias
float SelfTest[6];
float temperature;   // Stores the real internal chip temperature in Celsius
sCALIBRATION_ACAL calibrationDataACal;
sCALIBRATION_MCAL calibrationDataMCal;
float deltat = 0.0f;

static float MovingAveragePeriod = MOVING_AVERAGE / 2;
static float MxResultsFiltered[MOVING_AVERAGE]; 
static float MyResultsFiltered[MOVING_AVERAGE]; 
static float MzResultsFiltered[MOVING_AVERAGE];

static float mag_bias[3];
static float mag_scale[3];
static float mag_max[3];
static float mag_min[3];

unsigned char inCalibration = 0;

const float OrientationMatrix[6][9] = {
    // X_AXIS_DOWN
    {    0,    0,  1.0,   // x = chip z  
         0,  1.0,    0,   // y = chip y
      -1.0,    0,    0 }, // z = chip x (inverted)
    // Y_AXIS_DOWN
    {  1.0,    0,    0,   // x = chip x
         0,    0,  1.0,   // y = chip z
         0, -1.0,    0},  // z = chip y (inverted)
    // Z_AXIS_DOWN
    { -1.0,    0,    0,   // x = chip x (inverted)
         0,  1.0,    0,   // y = chip y
         0,    0, -1.0},  // z = chip z (inverted)
    // X_AXIS_UP
    {    0,    0, -1.0,   // x = chip z (inverted)
         0,  1.0,    0,   // y = chip y
       1.0,    0,    0},  // z = chip x
    // Y_AXIS_UP
    {  1.0,    0,    0,   // x = chip x
         0,    0, -1.0,   // y = chip z (inverted)
         0,  1.0,   0},   // z = chip y
    // Z_AXIS_UP
    {  1.0,    0,    0,   // x = chip x
         0,  1.0,    0,   // y = chip y
         0,    0,  1.0}   // z = chip z
};

const float magOrientationMatrix[6][9] = {
    // X_AXIS_DOWN
    {    0,    0, -1.0,   // x = chip z (inverted)
         0,  1.0,    0,   // y = chip y
       1.0,    0,    0 }, // z = chip x
    // Y_AXIS_DOWN                    
    {  1.0,    0,    0,   // x = chip x
         0,    0,  1.0,   // y = chip z
         0, -1.0,    0},  // z = chip y (inverted)
    // Z_AXIS_DOWN                    
    {  1.0,    0,    0,   // x = chip x
         0, -1.0,    0,   // y = chip y (inverted)
         0,    0, -1.0},  // z = chip z (inverted)
    // X_AXIS_UP                      
    {    0,    0,  1.0,   // x = chip z
         0,  1.0,    0,   // y = chip y
      -1.0,    0,    0},  // z = chip x (inverted)
    // Y_AXIS_UP                      
    {  1.0,    0,    0,   // x = chip x
         0,    0, -1.0,   // y = chip z (inverted)
         0,  1.0,    0},  // z = chip y
    // Z_AXIS_UP - ok                     
    {  1.0,    0,    0,   // x = chip x
         0,  1.0,    0,   // y = chip y 
         0,    0,  1.0}   // z = chip z
};

static BOOL dataAvailable = FALSE;
static BYTE intType = 0;
 
void getMres(void);
void getGres(void);
void getAres(void);
BOOL readBytes(BYTE address, BYTE subAddress, BYTE count, BYTE *dest);
BOOL readByte(BYTE, BYTE, BYTE *);
BOOL writeByte(BYTE, BYTE, BYTE);
BOOL write_Mag_Register(unsigned char mag_register, unsigned char value);
BOOL read_Mag_Register(unsigned char mag_register, BYTE *dest);
BOOL read_Mag_Registers(unsigned char mag_register, BYTE *dest);

static void MPU9250Handler(void) __irq 
{
	EXTINT = (1 << 3); 	// CLEAR EINT3
	if((IO_INT_STAT & 0x01) == 0x01) // there is a port 0 interrupt
	{
		// an interrupt from port 0
		if((IO0_INT_STAT_R & (1 << GPIO_PIN_MPU9250_INT)) == (1 << GPIO_PIN_MPU9250_INT))
		{
			// process the interrupt
			dataAvailable = TRUE;
			IO0_INT_CLR = (1 << GPIO_PIN_MPU9250_INT);
			if(intType == 0) intType = 1;		
		}

		if((IO0_INT_STAT_R & (1 << GPIO_PIN_RX0)) == (1 << GPIO_PIN_RX0))
		{
			// process the serial debug interrupt
			IO0_INT_CLR = (1 << GPIO_PIN_RX0);
			if(intType == 0) intType = 2;
		}	

		if((IO0_INT_STAT_R & (1 << GPIO_PIN_RX3)) == (1 << GPIO_PIN_RX3))
		{
			// process the serial debug interrupt
			IO0_INT_CLR = (1 << GPIO_PIN_RX3);
			if(intType == 0) intType = 3;
		}	
	}
	VICVectAddr = 0;		// Acknowledge Interrupt	
}


BOOL MPU9250()
{
	BYTE dummy;
	getMres();
	getAres();
	calibrationDataACal.boardOrientaton = BOARD_ORIENTATION_Z_AXIS_UP;

	BANK(0)
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80) == FALSE) return FALSE;
	TIME_DELAY(100) // Wait for all registers to reset 
	if(writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02) == FALSE) return FALSE; // to enable AK8963    
	if(readByte(MPU9250_ADDRESS, INT_STATUS_1, &dummy) == FALSE) return FALSE;

	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01) == FALSE) return FALSE;  // Auto select clock source to be PLL gyroscope reference if ready else
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x07) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, INT_ENABLE_1, 0x01) == FALSE) return FALSE;  // Enable data ready (bit 0) interrupt
	
	BANK(2)
	if(writeByte(MPU9250_ADDRESS, MOT_DETECT_CTRL, 0x00) == FALSE) return FALSE;

	// 00 110 00 1  BW 5.7Hz
	if(writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x31) == FALSE) return FALSE; // Write new ACCEL_CONFIG2 register value
	if(writeByte(MPU9250_ADDRESS, ACCEL_CONFIG_2, 0x02) == FALSE) return FALSE; // 2: Average 16 samples.
	if(writeByte(MPU9250_ADDRESS, ACCEL_SMPLRT_DIV_1, 0x00) == FALSE) return FALSE;  // 
	if(writeByte(MPU9250_ADDRESS, ACCEL_SMPLRT_DIV_2, 99) == FALSE) return FALSE;  // 11.25Hz

	BANK(3)
	writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, I2C_SLV1_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, I2C_SLV2_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, I2C_SLV3_CTRL, 0x00);
	
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x10); 
	writeByte(MPU9250_ADDRESS, I2C_MST_ODR_CONFIG, 0x04); 

	BANK(0)
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, LP_CONFIG, 0x40); 

	if(write_Mag_Register(AK8963_CNTL3, 0x01) == FALSE) return FALSE; // reset the device
	TIME_DELAY(5) // Wait for all registers to reset 

	if(write_Mag_Register(AK8963_CNTL2, 0x02) == FALSE) return FALSE; // Set magnetometer data resolution and sample ODR
	TIME_DELAY(5) // Wait for all registers to reset 
	
	setMovingAveragePeriod(MOVING_AVERAGE / 2);

	if(install_irq(EINT3_INT, MPU9250Handler, HIGHEST_PRIORITY ) == TRUE)
	{
		IO0_INT_EN_R = (1 << GPIO_PIN_MPU9250_INT); // P0.17 is the interrupt
		EXTMODE = (1 << EXTMODE3); // edge
		EXTPOLAR = (1 << EXTPOLAR3); // rising
	}
	
	return TRUE;
}

BOOL getReadReady(BOOL *WakeOnMotionFlag)
{
	BOOL returnValue = FALSE;
	
	*WakeOnMotionFlag = FALSE;
	
	if(dataAvailable == TRUE)
	{	
		BANK(0)
		if(readByte(MPU9250_ADDRESS, INT_STATUS_1, &returnValue) == FALSE) return FALSE;		
		
		if(intType == 1 && (returnValue & 0x04) == 0x04)
		{
			*WakeOnMotionFlag = 0x40;
			if(writeByte(MPU9250_ADDRESS, INT_STATUS, 0x00) == FALSE) return FALSE;
		}
		
		intType = 0;
		
		returnValue &= 0x01;
		dataAvailable = FALSE;
	}
	
	return returnValue;
}

void getMres(void)
{
	switch (Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
			mRes = 4912./8190.; // Proper scale to return uT
			break;
			
		case MFS_16BITS:
			mRes = 4912./32760.0; // Proper scale to return uT
			break;
	}
}


void getAres(void)
{
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			aRes = 2.0/32768.0;
			break;
			
		case AFS_4G:
			aRes = 4.0/32768.0;
			break;
		
		case AFS_8G:
			aRes = 8.0/32768.0;
			break;
			
		case AFS_16G:
			aRes = 16.0/32768.0;
			break;
	}
}

BOOL calibrationDataACalInit(BOOL checkValid)
{
	BOOL returnValue;
	
	if(((checkValid == TRUE && calibrationDataACal.dataValid == CALIBRATION_VALID) || (checkValid == FALSE)) &&
		calibrationDataACal.accelBias[0] > -10000 &&
		calibrationDataACal.accelBias[0] < 10000 &&
		calibrationDataACal.accelBias[1] > -10000 &&
		calibrationDataACal.accelBias[1] < 10000 &&
		calibrationDataACal.accelBias[2] > -10000 &&
		calibrationDataACal.accelBias[2] < 10000 &&
        calibrationDataACal.boardOrientaton <= BOARD_ORIENTATION_Z_AXIS_UP)
	{
		returnValue = TRUE;
	}
	else
	{
		calibrationDataACal.accelBias[0] = 0;
		calibrationDataACal.accelBias[1] = 0;
		calibrationDataACal.accelBias[2] = 0;
        calibrationDataACal.boardOrientaton = BOARD_ORIENTATION_Z_AXIS_UP;

		strcpy(calibrationDataACal.calInfo, "CAL INIT NOT SET");
		returnValue = FALSE;
	}
		
	return returnValue;
}

BOOL calibrationDataMCalInit(BOOL checkValid)
{
	BOOL returnValue;

	if(((checkValid == TRUE && calibrationDataMCal.dataValid == CALIBRATION_VALID) || (checkValid == FALSE)) &&
		calibrationDataMCal.magbiasInit[0] > -4000.0 &&
		calibrationDataMCal.magbiasInit[0] < 4000.0 &&
		calibrationDataMCal.magbiasInit[1] > -4000.0 &&
		calibrationDataMCal.magbiasInit[1] < 4000.0 &&
		calibrationDataMCal.magbiasInit[2] > -4000.0 &&
		calibrationDataMCal.magbiasInit[2] < 4000.0 &&
		calibrationDataMCal.magScaleInit[0] > -200.0 &&
		calibrationDataMCal.magScaleInit[0] < 200.0 &&
		calibrationDataMCal.magScaleInit[1] > -200.0 &&
		calibrationDataMCal.magScaleInit[1] < 200.0 &&
		calibrationDataMCal.magScaleInit[2] > -200.0 &&
		calibrationDataMCal.magScaleInit[2] < 200.0)
	{
		returnValue = TRUE;
	}
	else
	{
		calibrationDataMCal.magbiasInit[0] = 0.0;
		calibrationDataMCal.magbiasInit[1] = 0.0;
		calibrationDataMCal.magbiasInit[2] = 0.0;
		calibrationDataMCal.magScaleInit[0] = 1.0;
		calibrationDataMCal.magScaleInit[1] = 1.0;
		calibrationDataMCal.magScaleInit[2] = 1.0;
		calibrationDataMCal.Orientation[0] = 0;
		calibrationDataMCal.Orientation[1] = 0;
		calibrationDataMCal.Orientation[2] = 0;
		strcpy(calibrationDataMCal.calInfo, "CAL INIT NOT SET");
		returnValue = FALSE;
	}
		
	return returnValue;
}

void ApplyOrientation(float *xValue, float *yValue, float *zValue)
{
    float tempX, tempY, tempZ;
    
    tempX = OrientationMatrix[calibrationDataACal.boardOrientaton][0] * *xValue + OrientationMatrix[calibrationDataACal.boardOrientaton][1] * *yValue + OrientationMatrix[calibrationDataACal.boardOrientaton][2] * *zValue;
    tempY = OrientationMatrix[calibrationDataACal.boardOrientaton][3] * *xValue + OrientationMatrix[calibrationDataACal.boardOrientaton][4] * *yValue + OrientationMatrix[calibrationDataACal.boardOrientaton][5] * *zValue;
    tempZ = OrientationMatrix[calibrationDataACal.boardOrientaton][6] * *xValue + OrientationMatrix[calibrationDataACal.boardOrientaton][7] * *yValue + OrientationMatrix[calibrationDataACal.boardOrientaton][8] * *zValue;

    *xValue = tempX;
    *yValue = tempY;
    *zValue = tempZ;
}

void ApplyMagOrientation(float *xValue, float *yValue, float *zValue)
{
    float tempX, tempY, tempZ;
    
    tempX = magOrientationMatrix[calibrationDataACal.boardOrientaton][0] * *xValue + magOrientationMatrix[calibrationDataACal.boardOrientaton][1] * *yValue + magOrientationMatrix[calibrationDataACal.boardOrientaton][2] * *zValue;
    tempY = magOrientationMatrix[calibrationDataACal.boardOrientaton][3] * *xValue + magOrientationMatrix[calibrationDataACal.boardOrientaton][4] * *yValue + magOrientationMatrix[calibrationDataACal.boardOrientaton][5] * *zValue;
    tempZ = magOrientationMatrix[calibrationDataACal.boardOrientaton][6] * *xValue + magOrientationMatrix[calibrationDataACal.boardOrientaton][7] * *yValue + magOrientationMatrix[calibrationDataACal.boardOrientaton][8] * *zValue;

    *xValue = tempX;
    *yValue = tempY;
    *zValue = tempZ;
}

BOOL ReadDataandUpdateTime(void)
{
	// ST2 register stored here, must read ST2 at end of mag data acquisition
	BYTE rawData[8];  // x/y/z accel, mag register data stored here, temperature data
	BOOL returnValue = FALSE;
	BOOL WakeOnMotionFlag = FALSE;
		
	if(getReadReady(&WakeOnMotionFlag) == TRUE)
	{
		// accel, data availabe
		BANK(0)
		if(readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]) == FALSE) return FALSE;  // Read the six raw data registers into data array
		accelCount[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
		accelCount[1] = ((short)rawData[2] << 8) | rawData[3] ;  
		accelCount[2] = ((short)rawData[4] << 8) | rawData[5] ; 			

		// Now we'll calculate the accleration value into actual g's
		// This depends on scale being set
		ax = (float)accelCount[0] * aRes;
		ay = (float)accelCount[1] * aRes;
		az = (float)accelCount[2] * aRes;

		ApplyOrientation(&ax, &ay, &az);
		
		if(readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]) == FALSE) return FALSE;  // Read the two raw data registers sequentially into data array 

		tempCount = ((short)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
		// Temperature in degrees Centigrade
		temperature = ((float) tempCount) / 333.87 + 21.0;

	
		// Wait for magnetometer data ready bit to be set
		memset(rawData, 0, sizeof(rawData));
		if(read_Mag_Register(AK8963_ST1, &rawData[0]) == TRUE && (rawData[0] & 0x01) && read_Mag_Registers(AK8963_XOUT_L, &rawData[0]) == TRUE && (rawData[7] & 0x08) == 0)
		{	
			// Turn the MSB and LSB into a signed 16-bit value
			magCount[0] = ((short)rawData[1] << 8) | rawData[0];
			// Data stored as little Endian 
			magCount[1] = ((short)rawData[3] << 8) | rawData[2];
			magCount[2] = ((short)rawData[5] << 8) | rawData[4];

			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental
			// corrections
			// Get actual magnetometer value, this depends on scale being set
			mx = (float)magCount[0] * mRes;
			my = (float)magCount[1] * mRes;
			mz = (float)magCount[2] * mRes;

			utilityMovingAverage(MxResultsFiltered, MovingAveragePeriod, mx);
			utilityMovingAverage(MyResultsFiltered, MovingAveragePeriod, my);
			utilityMovingAverage(MzResultsFiltered, MovingAveragePeriod, mz);
			
			mx = MxResultsFiltered[0];
			my = MyResultsFiltered[0];
			mz = MzResultsFiltered[0];
			
			if(inCalibration == 0)
			{
				// Calculate the magnetometer values in milliGauss
				// Include factory calibration per data sheet and user environmental
				// corrections
				// Get actual magnetometer value, this depends on scale being set

				mx -= calibrationDataMCal.magbiasInit[0];
				my -= calibrationDataMCal.magbiasInit[1];
				mz -= calibrationDataMCal.magbiasInit[2];

				ApplyMagOrientation(&mx, &my, &mz);
			}
			else
			{
				if((inCalibration & 1) == 1)
				{
					if(mx > mag_max[0]) mag_max[0] = mx;
					if(mx < mag_min[0]) mag_min[0] = mx;
					if(my > mag_max[1]) mag_max[1] = my;
					if(my < mag_min[1]) mag_min[1] = my;
					if(mz > mag_max[2]) mag_max[2] = mz;
					if(mz < mag_min[2]) mag_min[2] = mz;
				}
			}
			
			// Calculate the time the last update took for use in the quaternion filters
			// Must be called before updating quaternions!
			Now = getTimerNow();

			// Set integration time by time elapsed since last filter update
			deltat = ((Now - lastUpdate) / 1000.0f);
			lastUpdate = Now;

			returnValue = TRUE;
		}
	}

	return returnValue;
}

// Function which accumulates accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer bias registers.
BOOL calibrateMPU9250(void)
{  
	BOOL returnValue;
	unsigned short ii, jj, packet_count, fifo_count, total_packets;
	unsigned short  accelsensitivity = 16384;  // = 16384 LSB/g
    BYTE data[6]; // data array to hold accelerometer x, y, z, data
    short accel_temp[3];
	BYTE dummy;
    
    calibrationDataACal.accelBias[0] = 0;
    calibrationDataACal.accelBias[1] = 0;
    calibrationDataACal.accelBias[2] = 0;

	// reset device
	// Write a one to bit 7 reset bit; toggle reset device
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80) == FALSE) return FALSE;
	TIME_DELAY(100)

	// get stable time source; Auto select clock source to be PLL gyroscope
	// reference if ready else use the internal oscillator, bits 2:0 = 001
	BANK(0)
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01) == FALSE) return FALSE;  
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00) == FALSE) return FALSE;
	TIME_DELAY(200)                              

	// Configure device for bias calculation
	if(writeByte(MPU9250_ADDRESS, INT_ENABLE_1, 0x00) == FALSE) return FALSE;   // Disable all interrupts
	if(writeByte(MPU9250_ADDRESS, FIFO_EN_1, 0x00) == FALSE) return FALSE;      // Disable FIFO
	if(writeByte(MPU9250_ADDRESS, FIFO_EN_2, 0x00) == FALSE) return FALSE;      // Disable FIFO
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00) == FALSE) return FALSE;   // Turn on internal clock source
	BANK(3)
	if(writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00) == FALSE) return FALSE; // Disable I2C master
	BANK(0)
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00) == FALSE) return FALSE;    // Disable FIFO and I2C master modes
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C) == FALSE) return FALSE;    // Reset FIFO and DMP
	TIME_DELAY(15)
  
	// Configure MPU6050 accelerometer for bias calculation
	BANK(2)
	if(writeByte(MPU9250_ADDRESS, ACCEL_SMPLRT_DIV_1, 0x00) == FALSE) return FALSE;  // 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
	if(writeByte(MPU9250_ADDRESS, ACCEL_SMPLRT_DIV_2, 0x00) == FALSE) return FALSE;  
	if(writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00) == FALSE) return FALSE; // Set accelerometer full-scale to 2 g, maximum sensitivity

    total_packets = 0;
	BANK(0)
    for (jj = 0; jj < 10; jj++)
    {
    	// Configure FIFO to capture accelerometer data for bias calculation
    	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40) == FALSE) return FALSE;   // Enable FIFO  
    	if(writeByte(MPU9250_ADDRESS, FIFO_EN_2, 0x10) == FALSE) return FALSE;     // Enable accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
		TIME_DELAY(40) // accumulate 40 samples in 40 milliseconds = 480 bytes
    
    	// At end of sample accumulation, turn off FIFO sensor read
    	if(writeByte(MPU9250_ADDRESS, FIFO_EN_2, 0x00) == FALSE) return FALSE;        // Disable accelerometer sensors for FIFO
    	if(readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]) == FALSE) return FALSE; // read FIFO sample count
    	fifo_count = ((unsigned short)data[0] << 8) | data[1];
    	packet_count = fifo_count/6;// How many sets of full accelerometer data for averaging
    
        for (ii = 0; ii < packet_count; ii++, total_packets++)
        {

    		if(readByte(MPU9250_ADDRESS, FIFO_R_W, &data[0]) == FALSE) return FALSE; // read data for averaging
    		if(readByte(MPU9250_ADDRESS, FIFO_R_W, &data[1]) == FALSE) return FALSE; // read data for averaging
    		if(readByte(MPU9250_ADDRESS, FIFO_R_W, &data[2]) == FALSE) return FALSE; // read data for averaging
     		if(readByte(MPU9250_ADDRESS, FIFO_R_W, &data[3]) == FALSE) return FALSE; // read data for averaging
    		if(readByte(MPU9250_ADDRESS, FIFO_R_W, &data[4]) == FALSE) return FALSE; // read data for averaging
    		if(readByte(MPU9250_ADDRESS, FIFO_R_W, &data[5]) == FALSE) return FALSE; // read data for averaging
			accel_temp[0] = (short) (((short)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
    		accel_temp[1] = (short) (((short)data[2] << 8) | data[3]  );
    		accel_temp[2] = (short) (((short)data[4] << 8) | data[5]  );
    
    		calibrationDataACal.accelBias[0] += (long) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    		calibrationDataACal.accelBias[1] += (long) accel_temp[1];
    		calibrationDataACal.accelBias[2] += (long) accel_temp[2];
    	}
        if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C) == FALSE) return FALSE;    // Reset FIFO and DMP
		TIME_DELAY(15)
	}

	calibrationDataACal.accelBias[0] /= (long) total_packets; // Normalize sums to get average count biases
	calibrationDataACal.accelBias[1] /= (long) total_packets;
	calibrationDataACal.accelBias[2] /= (long) total_packets;

    // automatically determine the direction
    calibrationDataACal.boardOrientaton = BOARD_ORIENTATION_Z_AXIS_UP;
    for (ii = 0; ii < 3; ii++)
    {
        if(calibrationDataACal.accelBias[ii] * (2.0 / 32768.0) > 0.500)
        {
            calibrationDataACal.boardOrientaton = (BOARD_ORIENTATION)(ii + 3);
            break;
        }

        if(calibrationDataACal.accelBias[ii] * (2.0 / 32768.0) < -0.500)
        {
            calibrationDataACal.boardOrientaton = (BOARD_ORIENTATION)ii;
            break;
        }
    }
    
    if(calibrationDataACal.accelBias[calibrationDataACal.boardOrientaton % 3] > 0L)
    {
        calibrationDataACal.accelBias[calibrationDataACal.boardOrientaton % 3] -= (long) accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        calibrationDataACal.accelBias[calibrationDataACal.boardOrientaton % 3] += (long) accelsensitivity;
    }
    
    returnValue = calibrationDataACalInit(FALSE);

	BANK(0)
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80) == FALSE) return FALSE;
	TIME_DELAY(100) // Wait for all registers to reset 
	if(writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02) == FALSE) return FALSE; // to enable AK8963    
	if(readByte(MPU9250_ADDRESS, INT_STATUS_1, &dummy) == FALSE) return FALSE;

	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01) == FALSE) return FALSE;  // Auto select clock source to be PLL gyroscope reference if ready else
	if(writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x07) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, INT_ENABLE_1, 0x01) == FALSE) return FALSE;  // Enable data ready (bit 0) interrupt
	
	BANK(2)
	if(writeByte(MPU9250_ADDRESS, MOT_DETECT_CTRL, 0x00) == FALSE) return FALSE;

	// 00 110 00 1  BW 5.7Hz
	if(writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x31) == FALSE) return FALSE; // Write new ACCEL_CONFIG2 register value
	if(writeByte(MPU9250_ADDRESS, ACCEL_CONFIG_2, 0x02) == FALSE) return FALSE; // 2: Average 16 samples.
	if(writeByte(MPU9250_ADDRESS, ACCEL_SMPLRT_DIV_1, 0x00) == FALSE) return FALSE;  // 
	if(writeByte(MPU9250_ADDRESS, ACCEL_SMPLRT_DIV_2, 99) == FALSE) return FALSE;  // 11.25Hz

	BANK(3)
	writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, I2C_SLV1_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, I2C_SLV2_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, I2C_SLV3_CTRL, 0x00);
	
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x10); 
	writeByte(MPU9250_ADDRESS, I2C_MST_ODR_CONFIG, 0x04); 

	BANK(0)
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00); 
	writeByte(MPU9250_ADDRESS, LP_CONFIG, 0x40); 

	return returnValue;
}
BOOL magcalMPU9250(float * dest1, float * dest2, MAGCAL_STATE magcalState) 
{
	float avg_rad;
	
	switch(magcalState)
	{
		case MAGCAL_STATE_INIT:
            mag_min[0] = 32767.0;
            mag_min[1] = 32767.0;
            mag_min[2] = 32767.0;
            mag_max[0] = -32767.0;
            mag_max[1] = -32767.0;
            mag_max[2] = -327670.0;
			write_Mag_Register(AK8963_CNTL2, 0x08);		
			break;
			
		case MAGCAL_STATE_FINISH:
			// Get hard iron correction
			mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2.0;  // get average x mag bias
			mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2.0;  // get average y mag bias
			mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2.0;  // get average z mag bias
 
            dest1[0] = mag_bias[0];  // save mag biases in uT for main program
            dest1[1] = mag_bias[1];   
            dest1[2] = mag_bias[2];  
			// Get soft iron correction estimate

			mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2.0;  // get average x axis max chord length
			mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2.0;  // get average y axis max chord length
			mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2.0;  // get average z axis max chord length

			avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0;

			dest2[0] = mag_scale[0] != 0 ? avg_rad / mag_scale[0] : 1.0;
			dest2[1] = mag_scale[1] != 0 ? avg_rad / mag_scale[1] : 1.0;
			dest2[2] = mag_scale[2] != 0 ? avg_rad / mag_scale[2] : 1.0;
			write_Mag_Register(AK8963_CNTL2, 0x02);
			break;
			
		default:;
	}
	
	return TRUE;
}

float getRate(void)
{
	float returnValue;
	
	returnValue = 1.0 / deltat;

	return returnValue;
}

BOOL read_Mag_Register(unsigned char mag_register, BYTE *dest)
{
	BANK(3)
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, 0x8C) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, mag_register) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81) == FALSE) return FALSE;
	BANK(0)
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x20) == FALSE) return FALSE; 
	TIME_DELAY(20) // sleep 60mS  20 ok 10 no 15 ok 12 no
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00) == FALSE) return FALSE; 
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_00, dest) == FALSE) return FALSE;
	BANK(3)
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0) == FALSE) return FALSE;
	BANK(0)

	return TRUE;
}

BOOL read_Mag_Registers(unsigned char mag_register, BYTE *dest)
{
	BANK(3)
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, 0x8C) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, mag_register) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x89) == FALSE) return FALSE;
	BANK(0)
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x20) == FALSE) return FALSE; 
	TIME_DELAY(20) // sleep 60mS
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00) == FALSE) return FALSE; 
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_00, &(dest[0])) == FALSE) return FALSE;
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_01, &(dest[1])) == FALSE) return FALSE;
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_02, &(dest[2])) == FALSE) return FALSE;
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_03, &(dest[3])) == FALSE) return FALSE;
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_04, &(dest[4])) == FALSE) return FALSE;
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_05, &(dest[5])) == FALSE) return FALSE;
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_06, &(dest[6])) == FALSE) return FALSE;
	if(readByte(MPU9250_ADDRESS, EXT_SLV_SENS_DATA_07, &(dest[7])) == FALSE) return FALSE;
	BANK(3)
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0) == FALSE) return FALSE;
	BANK(0)
	return TRUE;
}

BOOL write_Mag_Register(unsigned char mag_register, unsigned char value)
{
	BANK(3)
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, 0x0C) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, mag_register) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_DO, value) == FALSE) return FALSE;
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81) == FALSE) return FALSE;
	BANK(0)
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x20) == FALSE) return FALSE; 
	TIME_DELAY(60) // sleep 60mS
	if(writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00) == FALSE) return FALSE; 
	BANK(3)
	if(writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0) == FALSE) return FALSE;
	BANK(0)
	
	return TRUE;
}

BOOL getWhoAmI(BYTE *whoAmI9250, BYTE *whoamI8963)
{
	// Read the WHO_AM_I register, this is a good test of communication
	BANK(0)
	if(readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250, whoAmI9250) == FALSE) return FALSE;
	if(read_Mag_Register(WHO_AM_I_AK8963, whoamI8963) == FALSE) return FALSE;
	
	return TRUE;
}

// Wire.h read and write protocols
BOOL writeByte(BYTE address, BYTE subAddress, BYTE data)
{
	BOOL returnValue = FALSE;
	BYTE dest;

	returnValue = i2cDataTransaction(&data, 1, 0, address, subAddress, &dest);

	return returnValue;
}

BOOL readByte(BYTE address, BYTE subAddress, BYTE *dest)
{
	BOOL returnValue = FALSE;

	returnValue = i2cDataTransaction(0, 0, 1, address, subAddress, dest);

	return returnValue;
}

BOOL readBytes(BYTE address, BYTE subAddress, BYTE count, BYTE *dest)
{  
	BOOL returnValue = FALSE;

	returnValue = i2cDataTransaction(0, 0, count, address, subAddress, dest);

	return returnValue;
}

void setMovingAveragePeriod(float theMovingAveragePeriod)
{
    MovingAveragePeriod = theMovingAveragePeriod;
	memset(MyResultsFiltered, 0, sizeof(MyResultsFiltered));
}

void setAccelerometerFilter(float theAccelerometerFilter)
{
    AccelerometerFilter = theAccelerometerFilter;
}
float getMovingAveragePeriod(void)
{
    return MovingAveragePeriod;
}

float getAccelerometerFilter(void)
{
    return AccelerometerFilter;
}

void utilityMovingAverage(float *averageValues, float period, float value)
{
    BYTE thePeriod = period;

    memmove(&averageValues[1], &averageValues[0], thePeriod * 4);
    
    averageValues[0] = averageValues[1] + ((value - ((averageValues[1] + averageValues[thePeriod]) / 2.0)) / period);
}

