/*
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or
 a 3.3 V Teensy 3.1. We have disabled the internal pull-ups used by the Wire
 library in the Wire.h/twi.c utility file. We are also using the 400 kHz fast
 I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#ifndef _MPU9250_H_
#define _MPU9250_H_

// See also MPU-9250 Register Map and Descriptions, Revision 4.0,
// RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in above
// document; the MPU9250 and MPU9150 are virtually identical but the latter has
// a different register map

//Magnetometer Registers
#define WHO_AM_I_AK8963  0x01 // should return 0x09 // 0x48

#define AK8963_ST1       0x10  // data ready status bit 0
#define AK8963_XOUT_L    0x11  // data
#define AK8963_XOUT_H    0x12
#define AK8963_YOUT_L    0x13
#define AK8963_YOUT_H    0x14
#define AK8963_ZOUT_L    0x15
#define AK8963_ZOUT_H    0x16
#define AK8963_ST2       0x18  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL2     0x31  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL3     0x32  // Bit 0 is soft reset, returs to 0

#define SELF_TEST_X_GYRO   0x02 // BANK 1
#define SELF_TEST_Y_GYRO   0x03 // BANK 1
#define SELF_TEST_Z_GYRO   0x04 // BANK 1
#define SELF_TEST_X_ACCEL  0x0E // BANK 1
#define SELF_TEST_Y_ACCEL  0x0F // BANK 1
#define SELF_TEST_Z_ACCEL  0x10 // BANK 1
#define ACCEL_SMPLRT_DIV_1 0x10 // BANK 2
#define ACCEL_SMPLRT_DIV_2 0x11 // BANK 2
#define I2C_MST_ODR_CONFIG 0x00 // BANK 3
#define GYRO_SMPLRT_DIV    0x00 // BANK 2
#define GYRO_CONFIG_1      0x01 // BANK 2
#define GYRO_CONFIG_2      0x02 // BANK 2
#define ACCEL_CONFIG       0x14 // BANK 2
#define ACCEL_CONFIG_2     0x15 // BANK 2
#define LP_CONFIG          0x05 // BANK 0  
#define ACCEL_WOM_THR      0x13 // BANK 2
#define FIFO_EN_1          0x66 // BANK 0
#define FIFO_EN_2          0x67 // BANK 0
#define INT_PIN_CFG        0x0F // BANK 0
#define INT_ENABLE         0x10 // BANK 0
#define INT_ENABLE_1       0x11 // BANK 0
#define INT_STATUS         0x19 // BANK 0
#define INT_STATUS_1       0x1A // BANK 0
#define ACCEL_XOUT_H       0x2D // BANK 0
#define ACCEL_XOUT_L       0x2E // BANK 0
#define ACCEL_YOUT_H       0x2F // BANK 0
#define ACCEL_YOUT_L       0x30 // BANK 0
#define ACCEL_ZOUT_H       0x31 // BANK 0
#define ACCEL_ZOUT_L       0x32 // BANK 0
#define GYRO_XOUT_H        0x33 // BANK 0
#define GYRO_XOUT_L        0x34 // BANK 0
#define GYRO_YOUT_H        0x35 // BANK 0
#define GYRO_YOUT_L        0x36 // BANK 0
#define GYRO_ZOUT_H        0x37 // BANK 0
#define GYRO_ZOUT_L        0x38 // BANK 0
#define TEMP_OUT_H         0x39 // BANK 0
#define TEMP_OUT_L         0x3A // BANK 0
#define FIFO_EN_1		   0x66 // BANK 0
#define FIFO_EN_2		   0x67 // BANK 0
#define MOT_DETECT_CTRL    0x12// BANK 2
#define USER_CTRL          0x03 // BANK 0 // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1         0x06 // BANK 0 // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x07 // BANK 0
#define FIFO_COUNTH        0x70 // BANK 0
#define FIFO_COUNTL        0x70 // BANK 0
#define FIFO_R_W           0x72 // BANK 0
#define WHO_AM_I_MPU9250   0x00 // BANK 0 // Should return 0xEA //0x71
#define XA_OFFS_H          0x14 // BANK 1
#define XA_OFFS_L          0x15 // BANK 1
#define YA_OFFS_H          0x17 // BANK 1
#define YA_OFFS_L          0x18 // BANK 1
#define ZA_OFFS_H          0x1A // BANK 1
#define ZA_OFFS_L          0x1B // BANK 1
#define I2C_MST_ODR_CONFIG 0x00 // BANK 3
#define I2C_MST_CTRL       0x01 // BANK 3
#define I2C_MST_DELAY_CTRL 0x02 // BANK 3
#define I2C_SLV0_ADDR      0x03 // BANK 3
#define I2C_SLV0_REG       0x04 // BANK 3
#define I2C_SLV0_CTRL      0x05 // BANK 3
#define I2C_SLV0_DO        0x06 // BANK 3
#define I2C_SLV1_ADDR      0x07 // BANK 3
#define I2C_SLV1_REG       0x08 // BANK 3
#define I2C_SLV1_CTRL      0x09 // BANK 3
#define I2C_SLV1_DO        0x0A // BANK 3
#define I2C_SLV2_ADDR      0x0B // BANK 3
#define I2C_SLV2_REG       0x0C // BANK 3
#define I2C_SLV2_CTRL      0x0D // BANK 3
#define I2C_SLV2_DO        0x0E // BANK 3
#define I2C_SLV3_ADDR      0x0F // BANK 3
#define I2C_SLV3_REG       0x10 // BANK 3
#define I2C_SLV3_CTRL      0x11 // BANK 3
#define I2C_SLV3_DO        0x12 // BANK 3
#define I2C_SLV4_CTRL      0x15 // BANK 3
#define EXT_SLV_SENS_DATA_00 0x3B // BANK 0
#define EXT_SLV_SENS_DATA_01 0x3C // BANK 0
#define EXT_SLV_SENS_DATA_02 0x3D // BANK 0
#define EXT_SLV_SENS_DATA_03 0x3E // BANK 0
#define EXT_SLV_SENS_DATA_04 0x3F // BANK 0
#define EXT_SLV_SENS_DATA_05 0x40 // BANK 0
#define EXT_SLV_SENS_DATA_06 0x41 // BANK 0
#define EXT_SLV_SENS_DATA_07 0x42 // BANK 0
#define EXT_SLV_SENS_DATA_08 0x43 // BANK 0
#define BANK_SELECT		   0x7F

// Using the MPU-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define MPU9250_ADDRESS 0xD0  // Device address when ADO = 0
#define AK8963_ADDRESS  0x18   // Address of magnetometer

#define MAX_CAL_INFO_LENGTH 64 // this puts the EEPROM size to the end of calinfo to 100 bytes

#define DEFAULT_CAL_INFO "Default Calibration Info"
#define CALIBRATION_VALID 0xA55AD22D

//#define MOVING_AVERAGE 120
#define MOVING_AVERAGE 50

typedef enum
{
    BOARD_ORIENTATION_X_AXIS_DOWN,
    BOARD_ORIENTATION_Y_AXIS_DOWN,
    BOARD_ORIENTATION_Z_AXIS_DOWN,
    BOARD_ORIENTATION_X_AXIS_UP,
    BOARD_ORIENTATION_Y_AXIS_UP,
    BOARD_ORIENTATION_Z_AXIS_UP,
    BOARD_ORIENTATION_COUNT
} BOARD_ORIENTATION;

typedef enum
{
	MAGCAL_STATE_INIT,
	MAGCAL_STATE_FINISH,
	MAGCAL_STATE_COUNT
} MAGCAL_STATE;


// Set initial input parameters
typedef enum
{
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
} ASCALE;

typedef enum
{
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
} GSCALE;

typedef enum
{
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
} MSCALE;

typedef struct 
{
	// Bias corrections for gyro and accelerometer
	unsigned long dataValid;
	long  accelBias[3];
    BOARD_ORIENTATION boardOrientaton;
    char calInfo[MAX_CAL_INFO_LENGTH];
} sCALIBRATION_ACAL;

typedef struct 
{
	// Bias corrections for gyro and accelerometer
	unsigned long dataValid;
	float magbiasInit[3];
	float magScaleInit[3];
    float Orientation[3]; // Yaw, Pitch, Roll
	char calInfo[MAX_CAL_INFO_LENGTH];
} sCALIBRATION_MCAL;

BOOL MPU9250(void);
BOOL calibrateMPU9250(void);
BOOL ReadDataandUpdateTime(void);
BOOL magcalMPU9250(float * dest1, float * dest2, MAGCAL_STATE magcalState);
float getRate(void);
BOOL getWhoAmI(BYTE *whoAmI9250, BYTE *whoamI8963);
BOOL getReadReady(BOOL *WakeOnMotionFlag);
BOOL calibrationDataACalInit(BOOL checkValid);
BOOL calibrationDataMCalInit(BOOL checkValid);
void utilityMovingAverage(float *averageValues, float period, float value);
void setMovingAveragePeriod(float theMovingAveragePeriod);
void setAccelerometerFilter(float theAccelerometerFilter);
float getMovingAveragePeriod(void);
float getAccelerometerFilter(void);

#endif // _MPU9250_H_

