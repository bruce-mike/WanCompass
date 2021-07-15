#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

#define PI 3.141592654

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float *getQ(void);
void QuaternionInit(void);
							
#endif // _QUATERNIONFILTERS_H_
