#ifndef INC_AHRS_H_
#define INC_AHRS_H_
extern volatile float beta;
extern volatile float q0, q1, q2, q3;

void MadgwickAHRSupdate(float tdelta, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float tdelta, float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);
void quat2Euler( float q[4], float e[3] );

#endif /* INC_AHRS_H_ */
