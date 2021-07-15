#include <lpc23xx.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gpio.h"
#include "PLL.h"
#include "i2c.h" 
#include "serialDebug.h"
#include "serial.h"
#include "timer.h"
#include "MPU9250.h"
#include "storedconfig.h"
#include "compass.h"
#include "quaternionFilters.h"
#include "path.h"

#define NUMBER_OF_INITIAL_CAL_POSITIONS 6

#define MAXIMUM_RESEND_STRING_SIZE 256

#define RAD_TO_DEG (180.0 / PI)

//#define IDLE_TIMEOUT_TIME_MS 2000 // 2.0 seconds
#define IDLE_TIMEOUT_TIME_MS 6000 // 6.0 seconds
#define RESEND_TIMEOUT_TIME_MS 500 // 0.5 seconds

static TIMERCTL IdleTimeoutTimer;
static TIMERCTL ResendimeoutTimer;
static BYTE resendCount = 0;
static char resendString[MAXIMUM_RESEND_STRING_SIZE];

static BYTE commaFormat = FALSE;
static BYTE readContinuous = FALSE;
static unsigned long readUpdateCount = 0;
static unsigned long readUpdateDelta = 0;
static char *bMessage = 0;
static BYTE bMessageLength = 0;
float pitch, yaw, roll;
static float qw, qx, qy, qz;
static COMPASS_BOARD_STATE compassBoardState = COMPASS_BOARD_STATE_INIT;
static int currentDirection = 0;
static unsigned int setValue = 0;
static BYTE NumberOfInitialCalPositions = NUMBER_OF_INITIAL_CAL_POSITIONS;
static BOOL AutoCalFlag = FALSE;
static char calString[MAX_CAL_INFO_LENGTH];

static BOOL WakeOnMotionFlag = FALSE;

// Variables to hold latest sensor data values
extern float ax, ay, az, mx, my, mz;
// Factory mag calibration and mag bias
extern float SelfTest[6];
extern float temperature;   // Stores the real internal chip temperature in Celsius
extern sCALIBRATION_ACAL calibrationDataACal;
extern sCALIBRATION_MCAL calibrationDataMCal;
extern float deltat;
extern unsigned char inCalibration;
static BYTE index1 = 0;

float initAx, initAy, initAz;
float initMx, initMy, initMz;

static BOOL currentSpeed = FALSE;

void setSpeed(BOOL speed)
{
	if(currentSpeed != speed)
	{
		// don't change speeds while there is something in the FIFOs
		while((U0LSR & TEMT) == 0 || (U3LSR & TEMT) == 0)
		{
		}

		PLL_CONFIG(speed);
		timerSpeed(1, speed);
		serialDebugInitSetBaud(speed);
		serialInitSetBaud(speed);
		I2CSetBaud(speed);

		if(speed == TRUE)
		{
			timerShowLEDHeartbeat(250);
		}
		else
		{
			timerShowLEDHeartbeat(500);
		}
		
		currentSpeed = speed;
	}
}

static void resendMessage(char *theMessageToResend)
{
	resendCount = 0;
	while(resendCount++ < 10)
	{
		printf("%s", theMessageToResend);
		
		initTimer(&ResendimeoutTimer);
		startTimer(&ResendimeoutTimer, RESEND_TIMEOUT_TIME_MS);
		
		while(isTimerExpired(&ResendimeoutTimer) == FALSE)
		{
			serialDebugDoWork();
			serialDoWork();
		}
		
		hwPetWatchDog();
		
		if(serialGetQuitFlag() == TRUE || serialDebugGetQuitFlag() == TRUE)
		{
			compassBoardState = COMPASS_BOARD_STATE_IDLE;

			AutoCalFlag = FALSE;
			printf("ERRCAL, Exit Cal \n");
			break;
		}
		else if(serialGetMessage(&bMessage, &bMessageLength) == TRUE && bMessageLength > 0)
		{
			if(strncmp("ack", bMessage, 3) == 0)
			{
				break; // acknowledge received
			}
		}
	}
}

static void convertFromQuaternionsToEulerAngles()
{
	// Define output variables from updated quaternion---these are Tait-Bryan
	// angles, commonly used in aircraft orientation. In this coordinate system,
	// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
	// x-axis and Earth magnetic North (or TRUE North if corrected for local
	// declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the
	// Earth is positive, up toward the sky is negative. Roll is angle between
	// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
	// arise from the definition of the homogeneous rotation matrix constructed
	// from quaternions. Tait-Bryan angles as well as Euler angles are
	// non-commutative; that is, the get the correct orientation the rotations
	// must be applied in the correct order which for this configuration is yaw,
	// pitch, and then roll.
	// For more see
	// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// which has additional links.
	double t0, t1, t2, t3, t4, ysqr;
	qw = *(getQ() + 0);
	qx = *(getQ() + 1);
	qy = *(getQ() + 2);
	qz = *(getQ() + 3);
	ysqr = qy * qy;

	// roll (x-axis rotation)
	t0 = +2.0 * (qw * qx + qy * qz);
	t1 = +1.0 - 2.0 * (qx * qx + ysqr);
	roll = atan2(t0, t1);

	// pitch (y-axis rotation)
	t2 = +2.0 * (qw * qy - qz * qx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = asin(t2);

	// yaw (z-axis rotation)
	t3 = +2.0 * (qw * qz + qx * qy);
	t4 = +1.0 - 2.0 * (ysqr + qz * qz);
	yaw = atan2(t3, t4);

	yaw   *= RAD_TO_DEG;
	pitch *= RAD_TO_DEG;
	roll  *= RAD_TO_DEG;

	yaw += 360.0;
	if (yaw > 360.0)
	{
		yaw -= 360.0;
	}
}

static void printData()
{
   convertFromQuaternionsToEulerAngles();
	
	if(commaFormat == FALSE)
	{
		// Print temperature in degrees Centigrade
		printf("TEMP, %0.2f (°C) \n", temperature);

		// Print acceleration values in milligs!
		printf("ACCL, %0.2f aX (mg), %0.2f aY (mg), %0.2f aZ (mg) \n", 1000.0 * ax, 1000.0 * ay, 1000.0 * az);

		// Print gyro values in degree/sec
		printf("GYRO, 0 gX (deg/s), 0gY (deg/s), 0 gZ (deg/s) \n");

		// Print mag values in uT
		printf("MAG, %0.2f mH (uT), %0.2f mX (uT), %0.2f mY (uT), %0.2f mZ (uT), %0.2f mF (uT) \n",
			sqrt(mx * mx + my * my), mx, my, mz, sqrt(mx * mx + my * my + mz * mz));

		printf("QVAL, %0.2f qW, %0.2f qX, %0.2f qY, %0.2f qZ \n", qw, qx, qy, qz);

		// turn data upside down, keep 1° to 360°
 		yaw = 361.0 - yaw;
		
		printf("YPR, %0.2f yaw, %0.2f pitch, %0.2f roll \n", yaw, pitch, roll);

		printf("FREQ,%0.2f rate (Hz) \n", getRate());		
	}
	else
	{
		printf("%d, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, \n", 
		currentDirection,
		1000.0 * ax,
		1000.0 * ay,
		1000.0 * az,
		mx,
		my,
		mz,
		yaw,
		pitch,
		roll);
	}
}

static void printCal(void)
{
		
	// Initialize device for active mode read of acclerometer, gyroscope, and temperature
	printf("CALVALUE, %0.1f aX-axis (of factory),  %0.1f aY-axis (of factory),  %0.1f aZ-axis (of factory),  %0.1f gX-axis (within),  %0.1f gY-axis (within),  %0.1f gZ-axis (within),  0 mX-Axis : (sensitivity),  0 mY-Axis : (sensitivity),  0  mZ-Axis : (sensitivity)\n", 
		SelfTest[0],
		SelfTest[1],
		SelfTest[2],
		SelfTest[3],
		SelfTest[4],
		SelfTest[5]
		);
	
   // Print acceleration values!
	printf("agCal, %ld aX Bias, %ld aY Bias, %ld aZ Bias, 0 gX Bias, 0 gY Bias, 0 gZ Bias, %d Orientation, Info: %s \n",
		calibrationDataACal.accelBias[0],
		calibrationDataACal.accelBias[1],
		calibrationDataACal.accelBias[2],
		calibrationDataACal.boardOrientaton,
		calibrationDataACal.calInfo
		);

	// Print magBiasInit and magScaleInit in uT
	printf("magCalInit, %0.2f Init Bias 0 (uT), %0.2f Init Bias 1 (uT), %0.2f Init Bias 2 (uT), %0.2f Init Scale 0 (uT), %0.2f Init Scale 1 (uT), %0.2f Init Scale 2 (uT), %0.2f  OrientationYaw, %0.2f OrientationPitch, %0.2f OrientationRoll, Info: %s \n",
		calibrationDataMCal.magbiasInit[0],
		calibrationDataMCal.magbiasInit[1],
		calibrationDataMCal.magbiasInit[2],
		calibrationDataMCal.magScaleInit[0],
		calibrationDataMCal.magScaleInit[1],
		calibrationDataMCal.magScaleInit[2],
		calibrationDataMCal.Orientation[0],
		calibrationDataMCal.Orientation[1],
		calibrationDataMCal.Orientation[2],
		calibrationDataMCal.calInfo
		);

	printf("settings, %0.2f Moving Average Period, %0.2f Accelerometer Filter \n",
		getMovingAveragePeriod(),
		getAccelerometerFilter()
	);
}

BOOL compassRun(BOOL init)
{
	BYTE returnValue = FALSE;
	BYTE index;
	BYTE whoAmI9250;
	BYTE whoamI8963;
	static BYTE position = 0;
	static float declination = 0;
	static BOOL errorReset = FALSE;
	static int idleTimeoutCount = 0;
	
	if(serialGetQuitFlag() == TRUE || serialDebugGetQuitFlag() == TRUE)
	{
		if(compassBoardState >= COMPASS_BOARD_STATE_IDLE)
		{
			AutoCalFlag = FALSE;
			compassBoardState = COMPASS_BOARD_STATE_IDLE;
			resendMessage("ERRCAL, Exit Cal \n");
		}
	}

	switch(compassBoardState)
	{
		case COMPASS_BOARD_STATE_INIT:					
			if(init == TRUE)
			{
				printf("COMPASS INIT\n");
			}
			
			setSpeed(TRUE);
			
			compassBoardState = COMPASS_BOARD_STATE_BOARD_INIT;		
			break;
		
		case COMPASS_BOARD_STATE_BOARD_INIT:
			if(init == TRUE || errorReset == TRUE)
			{
				// Read the WHO_AM_I register, this is a good test of communication
				if(getWhoAmI(&whoAmI9250, &whoamI8963) == TRUE)
				{
					printf("STARTINFO, MPU9250 Who Am I; 0x%0.2X  MPU9250 (Should be 0xEA) \n", whoAmI9250);
					printf("STARTINFO, AK8963 Who Am I; 0x%0.2X  AK8963 (Should be 0x09) \n", whoamI8963);
				}
				else
				{
					printf("ERROR, Could not connect to MPU9250\n");			
					timerShowLEDHeartbeat(2000);
					compassBoardState = COMPASS_BOARD_STATE_ERROR;
					errorReset = FALSE;
				}
			}

			if(((init == TRUE || errorReset == TRUE) && whoAmI9250 == 0xEA && whoamI8963 == 0x09) || init == FALSE) // WHO_AM_I should always be 0x68
			{
				memcpy(&calibrationDataACal, storedConfigGetACAL(), sizeof(sCALIBRATION_ACAL));
				memcpy(&calibrationDataMCal, storedConfigGetMCAL(), sizeof(sCALIBRATION_MCAL));
				
				if(calibrationDataACalInit(TRUE) == TRUE)
				{
					if(init == TRUE) printf("STARTINFO, accel Cal Ok \n");
				}
				else
				{
					if(init == TRUE) printf("STARTINFO, accel Cal NOT Ok \n");
				}

				if(calibrationDataMCalInit(TRUE) == TRUE)
				{
					if(init == TRUE) printf("STARTINFO, Mag Init Cal Ok \n");
				}
				else
				{
					if(init == TRUE) printf("STARTINFO, Mag Init Cal NOT Ok \n");
				}

				compassBoardState = COMPASS_BOARD_STATE_IDLE;
			}
			else
			{
				printf("ERROR, Could not connect to MPU9250\n");			
				timerShowLEDHeartbeat(2000);
				compassBoardState = COMPASS_BOARD_STATE_ERROR;
				errorReset = FALSE;
			}
			break;
			
		case COMPASS_BOARD_STATE_IDLE:		
			// check if user input is available
			if(readContinuous == FALSE && getReadReady(&WakeOnMotionFlag) == TRUE && WakeOnMotionFlag == 0x40)
			{		
				setSpeed(TRUE);
				commaFormat = FALSE;
				
				printf("WakeOnMotion,\n");
				idleTimeoutCount = 0;
				compassBoardState = COMPASS_BOARD_STATE_IDLE_TIMEOUT;
				initTimer(&IdleTimeoutTimer);
				startTimer(&IdleTimeoutTimer, IDLE_TIMEOUT_TIME_MS);
			}
			
			if(readContinuous == TRUE && ReadDataandUpdateTime() == TRUE)
			{
				// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
				// the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
				// (+ up) of accelerometer and gyro! We have to make some allowance for this
				// orientationmismatch in feeding the output to the quaternion filter. For the
				// MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
				// along the x-axis just like in the LSM9DS0 sensor. This rotation can be
				// modified to allow any convenient orientation convention. This is ok by
				// aircraft orientation standards! Pass gyro rate as rad/s
				
				// MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, -mz, deltat);
				MahonyQuaternionUpdate(ax, ay, az, 0, 0, 0, mx, my, mz, deltat);
			}
			
            if(serialGetMessage(&bMessage, &bMessageLength) == TRUE && bMessageLength > 0)
            {
				bMessageLength = strlen(bMessage);

                if(bMessage[0] >= '0' && bMessage[0] <= '9') // then numeric
                {
                    bMessage[bMessageLength] = 0;
                    currentDirection = atoi(&bMessage[0]);
                    if(currentDirection > 360)
                    {
                        currentDirection = 0;
                    }
                }
                else if(strncmp("avg", bMessage, 3) == 0 && bMessage[3] >= '0' && bMessage[3] <= '9') // then numeric
                {
                    bMessage[bMessageLength] = 0;
                    setValue = atoi(&bMessage[3]);
                    if(setValue > MOVING_AVERAGE)
                    {
                        setValue = MOVING_AVERAGE;
                    }
                    setMovingAveragePeriod((float)setValue);
					printf("%d Moving Average Period\n", setValue);
                }
                else if(strncmp("cpos", bMessage, 4) == 0 && bMessage[4] >= '0' && bMessage[4] <= '9') // then numeric
                {
                    bMessage[bMessageLength] = 0;
                    setValue = atoi(&bMessage[4]);
                    NumberOfInitialCalPositions = setValue;
                    if(NumberOfInitialCalPositions > 2 * NUMBER_OF_INITIAL_CAL_POSITIONS)
                    {
                        NumberOfInitialCalPositions = NUMBER_OF_INITIAL_CAL_POSITIONS;
                    }
					sprintf(resendString, "Positions, %d  Number Of Initial Cal Positions\n", NumberOfInitialCalPositions);
					resendMessage(resendString);
				}
                else if(strncmp("auto", bMessage, 4) == 0)
                {				
					compassBoardState = COMPASS_BOARD_STATE_COMMAND_AUTO;
					setSpeed(TRUE);
                }
                else if(strncmp("read", bMessage, 4) == 0)
                {
                    readContinuous = FALSE;
                    commaFormat = FALSE;

					idleTimeoutCount = 0;
					compassBoardState = COMPASS_BOARD_STATE_IDLE_TIMEOUT;
					setSpeed(TRUE);
					initTimer(&IdleTimeoutTimer);
					startTimer(&IdleTimeoutTimer, IDLE_TIMEOUT_TIME_MS);
                }
                else if(strncmp("mon", bMessage, 3) == 0)
                {
                    readContinuous = TRUE;
                    commaFormat = FALSE;
					setSpeed(TRUE);
               }
                else if(strncmp("log", bMessage, 3) == 0)
                {
                    printf("Direction, ax, ay, az, mx, my, mz, yaw, pitch, roll, \n");
                    readContinuous = TRUE;
                    commaFormat = TRUE;
					setSpeed(TRUE);
               }
                else if(strncmp("stop", bMessage, 4) == 0)
                {
                    readContinuous = FALSE;
                    commaFormat = FALSE;
                }
                else if(strncmp("erase$Chip", bMessage, 10) == 0)
                {			
					setSpeed(TRUE);
					storedConfigEraseChip();
					compassBoardState = COMPASS_BOARD_STATE_BOARD_INIT;
					printf("Chip erased\n");
				}
                else if(strncmp("getinit", bMessage, 10) == 0)
                {			
					setSpeed(TRUE);
					printf("VERSION, %s, %s, %s, %s \n", __FILE__, THE_FILE_PATH1, __DATE__, __TIME__);
					
					if(getWhoAmI(&whoAmI9250, &whoamI8963) == TRUE)
					{
						printf("STARTINFO, MPU9250 Who Am I; 0x%0.2X  MPU9250 (Should be 0xEA) \n", whoAmI9250);
						printf("STARTINFO, AK8963 Who Am I; 0x%0.2X  AK8963 (Should be 0x09) \n", whoamI8963);

						memcpy(&calibrationDataACal, storedConfigGetACAL(), sizeof(sCALIBRATION_ACAL));
						memcpy(&calibrationDataMCal, storedConfigGetMCAL(), sizeof(sCALIBRATION_MCAL));
					
						if(calibrationDataACalInit(TRUE) == TRUE)
						{
							printf("STARTINFO, accel Cal Ok \n");
						}
						else
						{
							printf("STARTINFO, accel Cal NOT Ok \n");
						}

						if(calibrationDataMCalInit(TRUE) == TRUE)
						{
							printf("STARTINFO, Mag Init Cal Ok \n");
						}
						else
						{
							printf("STARTINFO, Mag Init Cal NOT Ok \n");
						}

						for(index = 0; index < strlen(calibrationDataACal.calInfo); index++)
						{
							if(calibrationDataACal.calInfo[index] < ' ')
							{
								calibrationDataACal.calInfo[index] = 0;
								break;
							}
						}
						
						for(index = 0; index < strlen(calibrationDataMCal.calInfo); index++)
						{
							if(calibrationDataMCal.calInfo[index] < ' ')
							{
								calibrationDataMCal.calInfo[index] = 0;
								break;
							}
						}
						
						printCal();

						printf("getinit\n");
					}
					else
					{
						printf("ERROR, getinit: MPU9250init\n");			
					}
				}
                else if(strncmp("erase$Cal", bMessage, 9) == 0)
                {			
					setSpeed(TRUE);
					// set the data to not valid
					calibrationDataACal.dataValid = ~CALIBRATION_VALID;
					storedConfigSetACAL(&calibrationDataACal);
					
					calibrationDataMCal.dataValid = ~CALIBRATION_VALID;
					storedConfigSetMCAL(&calibrationDataMCal);
					
					compassBoardState = COMPASS_BOARD_STATE_BOARD_INIT;
					
					printf("Cal erased\n");
				}
                else if(strncmp("getcal", bMessage, 6) == 0)
                {			
					for(index = 0; index < strlen(calibrationDataACal.calInfo); index++)
					{
						if(calibrationDataACal.calInfo[index] < ' ')
						{
							calibrationDataACal.calInfo[index] = 0;
							break;
						}
					}
					
 					for(index = 0; index < strlen(calibrationDataMCal.calInfo); index++)
					{
						if(calibrationDataMCal.calInfo[index] < ' ')
						{
							calibrationDataMCal.calInfo[index] = 0;
							break;
						}
					}
					
					printCal();
                }
            }
			
			if(readContinuous == TRUE)
			{
				readUpdateDelta = getTimerNow() - readUpdateCount;

				if (readUpdateDelta > 500) // update 500mS
				{
					readUpdateCount = getTimerNow();
					printData();
				}
			}
			break;
			
		case COMPASS_BOARD_STATE_IDLE_TIMEOUT:
			if(idleTimeoutCount >= MOVING_AVERAGE || isTimerExpired(&IdleTimeoutTimer))
			{			
				printData();
				compassBoardState = COMPASS_BOARD_STATE_IDLE;
			}
			else
			{
				if(ReadDataandUpdateTime() == TRUE)
				{
					idleTimeoutCount++;					
					// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
					// the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
					// (+ up) of accelerometer and gyro! We have to make some allowance for this
					// orientationmismatch in feeding the output to the quaternion filter. For the
					// MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
					// along the x-axis just like in the LSM9DS0 sensor. This rotation can be
					// modified to allow any convenient orientation convention. This is ok by
					// aircraft orientation standards! Pass gyro rate as rad/s
					
					// MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, -mz, deltat);
					MahonyQuaternionUpdate(ax, ay, az, 0, 0, 0, mx, my, mz, deltat);
					
					if(WakeOnMotionFlag == TRUE)
					{
						printf("WakeOnMotion,\n");
					}
				}
			}
			break;
			
		case COMPASS_BOARD_STATE_COMMAND_MCAL_INIT:
			// look for a quit message
			if(bMessageLength > 3 && AutoCalFlag == FALSE)
			{
				if((bMessageLength - 12)  > MAX_CAL_INFO_LENGTH) // 3 add 1 for 4 for null termination character
				{
					bMessageLength = MAX_CAL_INFO_LENGTH;
				}

				bMessage[bMessageLength] = 0;

                if(bMessage[bMessageLength - 1] < ' ')
                {
                    bMessage[bMessageLength - 1] = 0;
                }
				strcpy(calibrationDataMCal.calInfo, bMessage + 11);
			}
            else if(AutoCalFlag == FALSE)
			{
				strcpy(calibrationDataMCal.calInfo, DEFAULT_CAL_INFO);
			}
			else
			{
                strcpy(calibrationDataMCal.calInfo, "Sign Mag ");
                strcat(calibrationDataMCal.calInfo, calString);
			}

	
			if(magcalMPU9250((float *)&calibrationDataMCal.magbiasInit[0], (float *)&calibrationDataMCal.magScaleInit[0], MAGCAL_STATE_INIT) == FALSE)
			{
				AutoCalFlag = FALSE;
				resendMessage("ERRCAL, Mag Calibration NOT ok! \n");
				compassBoardState = COMPASS_BOARD_STATE_IDLE;
			}
			else
			{
				position = 0;
				inCalibration = 0;	
				index1 = 0;
				
				resendMessage("STARTCAL, Mag Calibration start! \n");
				sprintf(resendString, "Set to position, %d \n", position);
				resendMessage(resendString);

				compassBoardState = COMPASS_BOARD_STATE_COMMAND_MCAL_RUN;
			}
			break;
			
		case COMPASS_BOARD_STATE_COMMAND_MCAL_RUN:
			if(position < NumberOfInitialCalPositions)
			{
				if(inCalibration == 0)
				{
					if(serialGetMessage(&bMessage, &bMessageLength) == TRUE)
					{
						printf("  getting data for position, %d\n", position);
						inCalibration = 1;
					}
				}
				else
				{		
					if(ReadDataandUpdateTime() == TRUE)
					{
						if((inCalibration & 1) == 1)
						{
							index1++;			
						}
						
						if(index1 >= MOVING_AVERAGE)
						{			
							position++;
							index1 = 0;
							
							if(position < NumberOfInitialCalPositions)
							{			
								sprintf(resendString, "Set to position, %d \n", position);
								resendMessage(resendString);
							}
							else
							{		
								if(magcalMPU9250((float *)&calibrationDataMCal.magbiasInit[0], (float *)&calibrationDataMCal.magScaleInit[0], MAGCAL_STATE_FINISH) == TRUE && calibrationDataMCalInit(FALSE) == TRUE)
								{
									// generate Quaternions without gyro
									setMovingAveragePeriod(MOVING_AVERAGE / 2);
									compassBoardState = COMPASS_BOARD_STATE_COMMAND_MCAL_FINISH;
								}
								else
								{
									resendMessage("ERRCAL, Mag Calibration NOT ok! \n");
									AutoCalFlag = FALSE;
									compassBoardState = COMPASS_BOARD_STATE_IDLE;
								}
							}
							
							inCalibration = 0;
						}
					}
				}
			}
			break;
			
		case COMPASS_BOARD_STATE_COMMAND_MCAL_FINISH:	
			if(ReadDataandUpdateTime() == TRUE)
			{
				// run through several times to settle data
				MahonyQuaternionUpdate(ax, ay, az, 0, 0, 0, mx, my, mz, 0.1);		
				index1++;
			}
			
			if(index1 >= MOVING_AVERAGE)
			{			
				convertFromQuaternionsToEulerAngles();
				
				calibrationDataMCal.Orientation[0] = declination - yaw;
		
				calibrationDataMCal.Orientation[1] = pitch;
				calibrationDataMCal.Orientation[2] = roll;
				setMovingAveragePeriod(MOVING_AVERAGE / 2);
				
				if(calibrationDataMCal.Orientation[0] > 360.0)
				{
					calibrationDataMCal.Orientation[0] -= 360.0;
				}
				else if(calibrationDataMCal.Orientation[0] <= 0.0)
				{
					calibrationDataMCal.Orientation[0] += 360.0;
				}		
				
				for(index = 0; index < strlen(calibrationDataMCal.calInfo); index++)
				{
					if(calibrationDataMCal.calInfo[index] < ' ')
					{
						calibrationDataMCal.calInfo[index] = 0;
						break;
					}
				}
				
				calibrationDataMCal.dataValid = CALIBRATION_VALID;
				storedConfigSetMCAL(&calibrationDataMCal);
				
				resendMessage("ENDCAL, Mag Calibration done! \n");

				if(AutoCalFlag == TRUE)
				{
					printCal();
				}
				
				AutoCalFlag = FALSE;
				compassBoardState = COMPASS_BOARD_STATE_IDLE;
				returnValue = TRUE;
			}
			break;
			
		case COMPASS_BOARD_STATE_COMMAND_ACAL:
           if(bMessageLength > 4 && AutoCalFlag == FALSE)
            {
                if((bMessageLength - 12)  > MAX_CAL_INFO_LENGTH) // 3 add 1 for 4 for null termination character
                {
                    bMessageLength = MAX_CAL_INFO_LENGTH;
                }
                bMessage[bMessageLength] = 0;
                if(bMessage[bMessageLength - 1] < ' ')
                {
                    bMessage[bMessageLength - 1] = 0;
                }
                strcpy(calibrationDataACal.calInfo, bMessage + 11);
            }
            else if(AutoCalFlag == FALSE)
            {
                strcpy(calibrationDataACal.calInfo, DEFAULT_CAL_INFO);
            }
			else
			{
                strcpy(calibrationDataACal.calInfo, "Sign Accel ");
                strcat(calibrationDataACal.calInfo, calString);
			}
			// Calibrate gyro and accelerometers, load biases in bias registers
			if(calibrateMPU9250() == TRUE)
			{
				float tempMovingAverage = getMovingAveragePeriod();

				for(index = 0; index < strlen(calibrationDataACal.calInfo); index++)
				{
					if(calibrationDataACal.calInfo[index] < ' ')
					{
						calibrationDataACal.calInfo[index] = 0;
						break;
					}
				}
				
				calibrationDataACal.dataValid = CALIBRATION_VALID;
				storedConfigSetACAL(&calibrationDataACal);
				
				resendMessage("ENDACAL, Accel / GYRO Calibration done! \n");
				
				if(AutoCalFlag == FALSE)
				{
					compassBoardState = COMPASS_BOARD_STATE_IDLE;
				}
				else
				{
					compassBoardState = COMPASS_BOARD_STATE_COMMAND_MCAL_INIT;
				}
			}
			else
			{
				resendMessage("ERRCAL, Accel / GYRO Calibration not OK \n");

				if(AutoCalFlag == FALSE)
				{
					compassBoardState = COMPASS_BOARD_STATE_IDLE;
				}
				else
				{
					compassBoardState = COMPASS_BOARD_STATE_COMMAND_MCAL_INIT;
				}
			}
			break;
				
		case COMPASS_BOARD_STATE_COMMAND_AUTO:
			AutoCalFlag = TRUE;
			calString[0] = 0;
		
			bMessage[bMessageLength] = 0;
			if(bMessage[bMessageLength - 1] < ' ')
			{
				bMessage[bMessageLength - 1] = 0;
			}

			if(strlen(&bMessage[12]) > 0)
			{
				// get the cal message string
				strcpy(calString, &bMessage[12]);
			}
			
			if(bMessage[4] >= '0' && bMessage[4] <= '9') // then numeric
			{
				NumberOfInitialCalPositions = bMessage[4] & 0x0F;
			}
			
			declination = 0;
			bMessage[12] = 0;
			declination = atof(&bMessage[5]);
			declination /= 1000.0;
			compassBoardState = COMPASS_BOARD_STATE_COMMAND_ACAL;
			break;
				
		case COMPASS_BOARD_STATE_ERROR:
            if(serialGetMessage(&bMessage, &bMessageLength) == TRUE && bMessageLength > 0)
            {
				bMessageLength = strlen(bMessage);
                if(strncmp("auto", bMessage, 4) == 0)
                {				
					errorReset = TRUE;
					compassBoardState = COMPASS_BOARD_STATE_BOARD_INIT;
                }
			}
			break;
			
		default:;
	}
	
	return returnValue;
}

COMPASS_BOARD_STATE getCompassState(void)
{
	return compassBoardState;
}

BOOL getReadContinuous(void)
{
	return readContinuous;
}
